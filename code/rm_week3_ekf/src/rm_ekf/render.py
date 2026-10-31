"""可视化：四装甲板重投影、车体线框、HUD。"""
import math

import cv2
import numpy as np

from .armor import PLATE_NAMES, armor_object_points, chassis_box, plate_rt

COL_VIS = (80, 240, 120)
COL_HID = (110, 110, 110)
COL_BOX = (0, 200, 255)
PLATE_CODE = ("F", "L", "B", "R")


def project_world(pts_w, R_wc, K, dist):
    """世界系点 -> 像素（相机位于世界原点）。"""
    pc = np.asarray(pts_w, dtype=np.float64) @ R_wc
    img, _ = cv2.projectPoints(pc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, dist)
    return img.reshape(-1, 2)


def plate_visibility(p_w, yaw, length, width, cam_pos=None):
    if cam_pos is None:
        cam_pos = np.zeros(3)
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                   [math.sin(yaw), math.cos(yaw), 0.0],
                   [0.0, 0.0, 1.0]])
    out = []
    for k in range(4):
        R_vp, p_v = plate_rt(k, length, width)
        n_w = Rz @ R_vp[:, 2]
        c = p_w + Rz @ p_v
        out.append(float(n_w @ (cam_pos - c)))
    return out


def draw_vehicle(img, p_w, yaw, length, width, R_wc, K, dist, armor_w, armor_h,
                 cam_pos=None, thickness=2, label=True, color_vis=COL_VIS,
                 draw_box=True):
    """在图像上重投影四块装甲板与车体线框，返回每块板的可见性。"""
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                   [math.sin(yaw), math.cos(yaw), 0.0],
                   [0.0, 0.0, 1.0]])
    dots = plate_visibility(p_w, yaw, length, width, cam_pos)
    for k in range(4):
        R_vp, p_v = plate_rt(k, length, width)
        pts_v = armor_object_points(armor_w, armor_h) @ R_vp.T
        pts_w = pts_v @ Rz.T + (p_w + Rz @ p_v).reshape(1, 3)
        poly = project_world(pts_w, R_wc, K, dist)
        if not np.all(np.isfinite(poly)):
            continue
        ipts = poly.astype(np.int32)
        if dots[k] > 0:
            cv2.polylines(img, [ipts], True, color_vis, thickness, cv2.LINE_AA)
            if label:
                c = ipts.mean(axis=0)
                cv2.putText(img, PLATE_CODE[k], (int(c[0]) - 8, int(c[1]) + 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_vis, 1, cv2.LINE_AA)
        else:
            for i in range(4):
                cv2.line(img, tuple(ipts[i]), tuple(ipts[(i + 1) % 4]), COL_HID, 1, cv2.LINE_AA)
    if draw_box:
        pts_v, edges = chassis_box(length, width)
        pts_w = pts_v @ Rz.T + p_w.reshape(1, 3)
        proj = project_world(pts_w, R_wc, K, dist)
        for i, j in edges:
            cv2.line(img, tuple(proj[i].astype(int)), tuple(proj[j].astype(int)),
                     COL_BOX, 1, cv2.LINE_AA)
    return dots


def draw_hud(img, lines, origin=(14, 34), size=0.62, pad=9,
             color=(255, 255, 255), alpha=0.55):
    """左上角半透明底板 + 多行文本。"""
    overlay = img.copy()
    lh = int(size * 44)
    w = 0
    for s in lines:
        (tw, th), _ = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, size, 1)
        w = max(w, tw)
    x0, y0 = origin[0] - pad, origin[1] - int(size * 34)
    cv2.rectangle(overlay, (x0, y0), (x0 + w + 2 * pad, y0 + lh * len(lines) + pad), (0, 0, 0), -1)
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)
    y = origin[1]
    for s in lines:
        cv2.putText(img, s, (origin[0], y), cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)
        y += lh


def draw_detections(img, dets, color=(255, 200, 0)):
    for d in dets:
        c = np.asarray(d["corners"], dtype=np.float64).reshape(4, 2).astype(np.int32)
        cv2.polylines(img, [c], True, color, 1, cv2.LINE_AA)
