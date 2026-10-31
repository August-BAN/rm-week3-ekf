"""跟踪主流程：检测结果 -> 世界系观测 -> 整车 EKF -> 日志。"""
import math

import cv2
import numpy as np

from .armor import armor_object_points, plate_rt
from .geometry import wrap_angle

# 初始化时把首帧可见板当作车尾板（编号 2）：相机在车后追随，
# 可见板法线朝向相机 => 车头背离相机
INT_PLATE = 2


def _rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def det_to_world(det, R_wc):
    """把一帧检测（板心 + 板姿态）变到世界系。"""
    t_c = np.array(det["tvec"], dtype=np.float64).reshape(3)
    R_cp = np.array(det["R"], dtype=np.float64).reshape(3, 3)
    p_w = R_wc @ t_c
    n_w = R_wc @ R_cp[:, 2]
    n_w = n_w / max(np.linalg.norm(n_w), 1e-9)
    return p_w, n_w, t_c, R_cp


def plate_corners_px(ekf, k, R_wc, cfg):
    """用当前状态预测板 k 的四角点像素。"""
    pk, _ = ekf.plate_state(k)
    R_vp, _ = plate_rt(k, cfg.vehicle_length, cfg.vehicle_width)
    R_wp = _rotz(ekf.x[6]) @ R_vp
    pts_w = armor_object_points(cfg.armor_w, cfg.armor_h) @ R_wp.T + pk.reshape(1, 3)
    pc = pts_w @ R_wc
    img, _ = cv2.projectPoints(pc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), cfg.K, cfg.dist)
    return img.reshape(-1, 2)


def associate(ekf, zs, allow_jump=1, switch_ratio=0.55, dphi_keep=0.45, dp_keep=0.30):
    """关联：带迟滞的板编号维护。

    车体近似四重对称，单板观测只能把偏航定到 90° 的整数倍；
    若每帧都按最近邻重选板编号，换编号会被误当成"整车又转了 90°"，
    造成偏航持续漂移（实测可达 250°/s）。故：
      1) 优先沿用上一帧编号；
      2) 仅当上一编号残差明显变大（偏航 > 26° 或位置 > 0.3 m）
         且备选编号残差不到其 55% 时才切换；
      3) 每次只允许 ±1 块板。
    """
    if not ekf.initialized:
        return []
    prev = ekf.last_plate if ekf.last_plate >= 0 else 0
    cands = [(prev + d) % 4 for d in range(-allow_jump, allow_jump + 1)]
    out = []
    used = set()
    for di, (z, aux) in enumerate(zs):
        scored = []
        for k in cands:
            if k in used:
                continue
            pk, yawk = ekf.plate_state(k)
            dp = float(np.linalg.norm(z[0:3] - pk))
            dphi = float(abs(wrap_angle(z[3] - yawk)))
            scored.append((dp + 0.35 * dphi, k, dp, dphi))
        if not scored:
            continue
        scored.sort()
        cur = next((s for s in scored if s[1] == prev), None)
        best = scored[0]
        if cur is None or cur is best:
            pick = best
        elif cur[2] > dp_keep or cur[3] > dphi_keep:
            pick = best if best[0] < switch_ratio * cur[0] else cur
        else:
            pick = cur
        used.add(pick[1])
        out.append((di, pick[1], pick[0]))
    return out


def run_frame(ekf, dets, R_wc, cfg, dt, log_rec, params):
    """处理一帧：预测 -> 关联 -> 更新。"""
    ekf.predict(dt)
    obs = []
    for det in dets:
        p_w, n_w, t_c, R_cp = det_to_world(det, R_wc)
        depth = float(t_c[2])
        z_yaw = math.atan2(n_w[1], n_w[0])
        expected_w = cfg.K[0, 0] * cfg.armor_w / max(depth, 1e-3)
        cos_theta = float(np.clip(det.get("width_px", expected_w) / max(expected_w, 1e-6), 0.15, 1.0))
        obs.append((np.concatenate([p_w, [z_yaw]]),
                    {"depth": depth, "cos": cos_theta, "det": det,
                     "p_w": p_w, "n_w": n_w, "corners": det["corners"]}))
    summary = {"n_det": len(obs), "updates": [], "init": False}
    if obs and (not ekf.initialized or ekf.miss > params.reinit_miss):
        # 用最"正视"的一块板初始化
        best = max(obs, key=lambda o: o[1]["cos"])
        z = best[0]
        ekf.initialize(INT_PLATE, z[0:3], z[3])
        summary["init"] = True
        summary["updates"].append({"plate": INT_PLATE, "chi2": 0.0, "init": True,
                                   "accepted": True, "residual": [0, 0, 0, 0]})
    if ekf.initialized:
        for di, k, cost in associate(ekf, obs):
            z, aux = obs[di]
            pred_px = plate_corners_px(ekf, k, R_wc, cfg)
            meas = np.asarray(aux["corners"], dtype=np.float64).reshape(4, 2)
            reproj_px = float(np.mean(np.linalg.norm(pred_px - meas, axis=1)))
            R = ekf.measurement_cov(k, z[0:3], R_wc[:, 2], aux["depth"], aux["cos"])
            res = ekf.update_plate(k, z, R)
            if res is None:
                continue
            summary["updates"].append({
                "plate": k, "chi2": res["chi2"], "accepted": res["accepted"],
                "is_switch": res.get("is_switch", False),
                "cos": aux["cos"], "depth": aux["depth"],
                "reproj_px": reproj_px,
                "residual": np.asarray(res["residual"]).tolist()})
    log_rec.update({
        "init": summary["init"],
        "n_det": len(obs),
        "updates": summary["updates"],
        "initialized": ekf.initialized,
        "state": ekf.x.tolist(),
        "std": np.sqrt(np.diag(ekf.P)).tolist(),
        "miss": ekf.miss,
        "plate": ekf.last_plate,
        "dets_world": [{"p": o[1]["p_w"].tolist(), "yaw": float(o[0][3]),
                        "depth": o[1]["depth"], "cos": o[1]["cos"],
                        "corners": o[1]["corners"], "score": o[1]["det"]["score"]}
                       for o in obs],
    })
    return summary
