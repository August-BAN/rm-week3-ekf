"""传统视觉装甲板检测：颜色分割 -> 灯条筛选 -> 几何配对 -> 打分+NMS。

设计沿用第二周作业：装甲板不整体检测，而先找"稳定可分割"的灯条，
再用全部为比值的几何约束两两配对，因此同一套阈值可同时适配远近目标。
"""
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class LightBar:
    corners: np.ndarray      # (4,2) float32, 按几何顺序
    center: np.ndarray       # (2,)
    length: float            # 长边
    width: float             # 短边
    axis: np.ndarray         # (2,) 长轴单位向量，方向统一为"指向图像下方"
    end_a: np.ndarray        # (2,) 长轴起点（上端）
    end_b: np.ndarray        # (2,) 长轴终点（下端）
    area: float


@dataclass
class Detection:
    corners: np.ndarray                  # (4,2) float32: 左上/左下/右下/右上
    score: float
    center: np.ndarray                   # (2,)
    bars: tuple = field(default=())      # 两个 LightBar
    extra: dict = field(default=dict)    # 交给解算器/跟踪器的附加信息

    @property
    def width_px(self):
        return float(np.linalg.norm(self.corners[3] - self.corners[0]))


class DetectorConfig:
    def __init__(self, **kw):
        # 颜色差分阈值（R - max(G,B)），实拍橙色灯条
        self.diff_thresh = 40
        self.use_close = True
        self.close_ksize = 3
        self.min_area = 40.0
        self.max_len = 320.0
        self.min_len = 9.0
        self.max_width = 70.0
        self.min_aspect = 1.6
        self.max_aspect = 16.0
        self.max_tilt_deg = 42.0
        # 配对约束（全为比值）
        self.pair_max_angle = 25.0
        self.pair_h_ratio = (0.55, 1.9)
        self.pair_top_tol = 0.55
        self.pair_bottom_tol = 0.55
        self.pair_d_over_h = (0.40, 4.60)
        # 灯条间距/灯条长 的先验（由近正视帧实测得到的平均值）
        self.aspect_prior = 2.55
        self.aspect_sigma = 1.05
        self.nms_iou = 0.25
        self.max_dets = 8
        self.min_score = 0.18
        for k, v in kw.items():
            setattr(self, k, v)


def _bar_from_rect(rect_pts):
    """由 4 个顶点（任意顺序）构造 LightBar。"""
    pts = np.asarray(rect_pts, dtype=np.float64).reshape(4, 2)
    c = pts.mean(axis=0)
    ang = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    order = np.argsort(ang)
    pts = pts[order]
    # 找两条相邻边的长度，确定长短轴
    d = np.roll(pts, -1, axis=0) - pts
    lens = np.linalg.norm(d, axis=1)
    i = int(np.argmax(lens))          # 长边
    j = (i + 1) % 4                   # 短边
    u_long = d[i] / max(lens[i], 1e-9)
    u_short = d[j] / max(lens[j], 1e-9)
    length = float(lens[i])
    width = float(lens[j])
    # 长轴方向统一向下（图像 y 增大方向）
    if u_long[1] < 0:
        u_long = -u_long
    proj = (pts - c) @ u_long
    a = pts[int(np.argmin(proj))]
    b = pts[int(np.argmax(proj))]
    # 端点偏移到长轴线上
    pa = c + u_long * float(np.min(proj))
    pb = c + u_long * float(np.max(proj))
    return LightBar(corners=pts.astype(np.float32), center=c, length=length,
                    width=width, axis=u_long, end_a=pa, end_b=pb,
                    area=float(length * width)), a, b


def _split_mask(bgr, cfg):
    b = bgr[:, :, 0].astype(np.int16)
    g = bgr[:, :, 1].astype(np.int16)
    r = bgr[:, :, 2].astype(np.int16)
    diff = r - np.maximum(g, b)
    mask = (diff > cfg.diff_thresh).astype(np.uint8) * 255
    if cfg.use_close:
        k = np.ones((cfg.close_ksize, cfg.close_ksize), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def find_light_bars(bgr, cfg: DetectorConfig):
    mask = _split_mask(bgr, cfg)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    bars = []
    for c in cnts:
        if cv2.contourArea(c) < cfg.min_area:
            continue
        rect = cv2.minAreaRect(c)
        (cx, cy), (w, h), _ = rect
        if min(w, h) < 1.2:
            continue
        pts = cv2.boxPoints(rect)
        bar, pa, pb = _bar_from_rect(pts)
        if bar.length < cfg.min_len or bar.length > cfg.max_len:
            continue
        if bar.width < 1.2 or bar.width > cfg.max_width:
            continue
        aspect = bar.length / max(bar.width, 1e-6)
        if aspect < cfg.min_aspect or aspect > cfg.max_aspect:
            continue
        tilt = np.degrees(np.arctan2(abs(bar.axis[0]), abs(bar.axis[1])))
        if tilt > cfg.max_tilt_deg:
            continue
        bars.append(bar)
    return mask, bars


def _score_pair(bi: LightBar, bj: LightBar, cfg: DetectorConfig):
    """返回 (score, corners) 或 (0, None)。"""
    hi, hj = bi.length, bj.length
    mean_h = 0.5 * (hi + hj)
    ratio = min(hi, hj) / max(hi, hj)
    lo, hi_r = cfg.pair_h_ratio
    if ratio < lo or ratio > hi_r:
        return 0.0, None
    ang = np.degrees(np.arccos(np.clip(abs(float(np.dot(bi.axis, bj.axis))), -1, 1)))
    if ang > cfg.pair_max_angle:
        return 0.0, None
    # 端点在长轴方向上的高度差（两灯条应上下端对齐）
    top_err = min(abs(float(np.dot(bi.axis, bi.end_a - bj.end_a))),
                  abs(float(np.dot(bi.axis, bi.end_a - bj.end_b)))) / mean_h
    bot_err = min(abs(float(np.dot(bi.axis, bi.end_b - bj.end_b))),
                  abs(float(np.dot(bi.axis, bi.end_b - bj.end_a)))) / mean_h
    if top_err > cfg.pair_top_tol or bot_err > cfg.pair_bottom_tol:
        return 0.0, None
    d = float(np.linalg.norm(bi.center - bj.center))
    if abs(float(bi.center[0] - bj.center[0])) < 0.5 * (bi.width + bj.width):
        return 0.0, None
    dro = d / mean_h
    dlo, dhi = cfg.pair_d_over_h
    if dro < dlo or dro > dhi:
        return 0.0, None

    def rbf(x, sigma):
        return float(np.exp(-0.5 * (x / sigma) ** 2))

    s = (rbf(1.0 - ratio, 0.30) * rbf(ang / 25.0, 0.45) *
         rbf(top_err, 0.30) * rbf(bot_err, 0.30) *
         rbf(dro - cfg.aspect_prior, cfg.aspect_sigma))
    if s <= 0.0:
        return 0.0, None
    corners = _pair_corners(bi, bj)
    if corners is None:
        return 0.0, None
    return s, corners


def _bar_outer_points(bar: LightBar, other_center):
    """灯条"外侧"上下端点（远离另一根灯条的一侧）。"""
    c = bar.center
    d = np.asarray(other_center) - c
    lat = np.array([-bar.axis[1], bar.axis[0]])
    if np.dot(lat, d) < 0:
        lat = -lat           # lat 指向另一根灯条
    top = bar.end_a - lat * (bar.width * 0.5)
    bot = bar.end_b - lat * (bar.width * 0.5)
    return top, bot


def _pair_corners(bi: LightBar, bj: LightBar):
    if bi.center[0] <= bj.center[0]:
        left, right = bi, bj
    else:
        left, right = bj, bi
    lt, lb = _bar_outer_points(left, right.center)
    rt, rb = _bar_outer_points(right, left.center)
    if lt[1] > lb[1]:
        lt, lb = lb, lt
    if rt[1] > rb[1]:
        rt, rb = rb, rt
    # 板高应远大于灯条宽度
    if abs(lb[1] - lt[1]) < 6 or abs(rb[0] - lt[0]) < 6:
        return None
    return np.array([lt, lb, rb, rt], dtype=np.float32)


def _nms(dets, iou_th):
    if not dets:
        return []
    boxes = []
    for d in dets:
        x0, y0 = d.corners.min(axis=0)
        x1, y1 = d.corners.max(axis=0)
        boxes.append([float(x0), float(y0), float(x1 - x0), float(y1 - y0)])
    idx = cv2.dnn.NMSBoxes(boxes, [d.score for d in dets], 0.0, iou_th)
    if len(idx) == 0:
        return []
    return [dets[int(i)] for i in np.array(idx).reshape(-1)]


def detect(bgr, cfg: DetectorConfig | None = None):
    """返回 (mask, detections)。detections 按得分降序。"""
    cfg = cfg or DetectorConfig()
    mask, bars = find_light_bars(bgr, cfg)
    cands = []
    n = len(bars)
    for i in range(n):
        for j in range(i + 1, n):
            if abs(float(bars[i].center[0] - bars[j].center[0])) < 1.0:
                continue
            s, corners = _score_pair(bars[i], bars[j], cfg)
            if s <= cfg.min_score or corners is None:
                continue
            c = corners.mean(axis=0)
            cands.append(Detection(corners=corners, score=float(s), center=c,
                                   bars=(bars[i], bars[j])))
    cands.sort(key=lambda d: -d.score)
    # 贪心：一根灯条只属于一块板
    used = set()
    keep = []
    for d in cands:
        ids = (id(d.bars[0]), id(d.bars[1]))
        if ids[0] in used or ids[1] in used:
            continue
        used.update(ids)
        keep.append(d)
    keep = _nms(keep, cfg.nms_iou)
    return mask, keep[: cfg.max_dets]
