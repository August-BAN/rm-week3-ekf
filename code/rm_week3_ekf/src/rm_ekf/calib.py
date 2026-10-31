"""相机-IMU 外参标定。

思路见 est_extrinsic：装甲板 y 轴 = 车体上方向 ≈ 重力反方向，
PnP 给出 u_c（世界上方向在相机系），IMU 给出 z_b（同一方向在机体系）。
Kabsch 配准 u_c ≈ R_bc^T z_b 求解 R_bc 的 roll/pitch；
绕重力轴的偏航由"相机光轴朝车头"的先验补齐（重力观测对其不可观）。
"""
import json
import math

import numpy as np

from .geometry import euler_zyx


def kabsch(a_vecs, b_vecs, weights=None):
    """求 R 使 R @ a ≈ b（加权最小二乘 + SO(3) 投影）。"""
    a = np.asarray(a_vecs, dtype=np.float64).reshape(-1, 3)
    b = np.asarray(b_vecs, dtype=np.float64).reshape(-1, 3)
    w = np.ones(len(a)) if weights is None else np.asarray(weights, dtype=np.float64).reshape(-1)
    M = (b * w[:, None]).T @ a
    U, _, Vt = np.linalg.svd(M)
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    return U @ D @ Vt


def estimate_R_bc(up_cam, up_body, weights=None, yaw_prior_axis=(1.0, 0.0, 0.0)):
    """up_cam: 世界上方向在相机系的观测 (N,3); up_body: 同一方向在机体系 (N,3)。"""
    A0 = kabsch(up_body, up_cam, weights)      # A0 @ z_b ≈ u_c, A0 = R_bc^T 的近似
    R0 = A0.T
    # 用 φ 绕机体系 z 轴旋转，使相机光轴在水平面上最接近机体系 x（车头）
    ax = np.asarray(yaw_prior_axis, dtype=np.float64)
    ax = ax / np.linalg.norm(ax)
    best_phi, best_score = 0.0, -2.0
    for phi in np.linspace(0.0, 2 * math.pi, 721):
        c, s = math.cos(phi), math.sin(phi)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        R = Rz @ R0
        fwd = R[:, 2].copy()              # 相机光轴在机体系
        fwd[2] = 0.0
        n = np.linalg.norm(fwd)
        if n < 1e-9:
            continue
        score = float((fwd / n) @ ax)
        if score > best_score:
            best_score, best_phi = score, phi
    c, s = math.cos(best_phi), math.sin(best_phi)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    R_bc = Rz @ R0
    pred = (R_bc.T @ np.asarray(up_body, dtype=np.float64).T).T
    err = np.degrees(np.arccos(np.clip(np.sum(pred * np.asarray(up_cam), axis=1), -1, 1)))
    info = {
        "R_bc": R_bc.tolist(),
        "euler_zyx_deg": [math.degrees(v) for v in euler_zyx(R_bc)],
        "up_residual_rms_deg": float(np.sqrt(np.mean(err ** 2))),
        "up_residual_max_deg": float(err.max()),
        "yaw_phi_deg": math.degrees(best_phi),
        "n_samples": int(len(err)),
    }
    return R_bc, info


def plate_up_observations(detections, imu, src="body_to_world", min_score=0.0):
    """从检测结果构造 (u_c, z_b, w)。"""
    uc, zb, w = [], [], []
    for r in detections["frames"]:
        R_wb = imu.R_of_frame(r["frame"], src=src)
        z_b = R_wb.T @ np.array([0.0, 0.0, 1.0])
        for det in r["detections"]:
            if not det["ok"]:
                continue
            if det["score"] < min_score:
                continue
            R_cp = np.array(det["R"], dtype=np.float64).reshape(3, 3)
            u = R_cp[:, 1]
            n = np.linalg.norm(u)
            if n < 1e-9:
                continue
            uc.append(u / n)
            zb.append(z_b)
            # 板越大, 姿态越可信
            w.append(max(det.get("width_px", 50.0), 20.0) ** 2)
    return np.array(uc), np.array(zb), np.array(w)


def save_extrinsic(path, R_bc, info):
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"R_bc": np.asarray(R_bc).tolist(), "info": info}, f, ensure_ascii=False, indent=2)


def load_extrinsic(path):
    with open(path, encoding="utf-8") as f:
        d = json.load(f)
    return np.array(d["R_bc"], dtype=np.float64), d.get("info", {})
