"""整车 EKF：状态 = [p(3), v(3), yaw, vyaw]，观测 = 装甲板中心位置 + 板法线偏航。

运动模型：世界系匀速（CV）+ 偏航匀角速度，连续白噪声加速度驱动，
         离散化后位置与速度的噪声块互相耦合（保证协方差形状正确）。
观测模型：路面平坦假设下车体只有 roll/pitch≈0 的小量，板心 h(x)=p+Rz(yaw)p_k^V，
         观测量由 PnP 结果经 IMU 姿态旋到世界系得到，噪声随距离与板的正视程度自适应。
"""
import math

import numpy as np

from .armor import PLATE_NORMALS, plate_rt, plate_yaw_offset
from .geometry import wrap_angle


class EKFParams:
    def __init__(self, **kw):
        self.q_acc = 1.5          # 加速度噪声谱密度 m^2/s^3
        self.q_alpha = 2.0        # 角加速度噪声谱密度 rad^2/s^3
        self.r_pos_lat = 0.02     # 横向位置噪声底 m
        self.r_pos_depth = 0.05   # 深度方向噪声底 m
        self.r_pos_range = 0.02   # 与距离成正比的部分 (m per m)
        self.r_yaw_base = 1.5     # 偏航噪声底 (deg)
        self.r_yaw_foreshort = 6.0  # 板面越斜, 偏航噪声越大 (deg)
        self.gate_chi2 = 16.27    # 4 自由度 99.7%
        self.max_miss = 15        # 连续丢失多少帧后认为目标丢失
        self.reinit_miss = 25
        self.max_pos_step = 0.6   # 单次更新位置修正上限 m
        self.switch_inflate = 4.0  # 切换可见板时偏航噪声放大倍数
        self.ground_constraint = 1  # 地面约束：垂向速度恒为 0
        for k, v in kw.items():
            setattr(self, k, v)


def _rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class VehicleEKF:
    """单个目标车的 EKF。state: [x,y,z,vx,vy,vz,yaw,vyaw]"""

    N = 8

    def __init__(self, length, width, params: EKFParams):
        self.length = float(length)
        self.width = float(width)
        self.p = params
        self.x = np.zeros(self.N)
        self.P = np.eye(self.N) * 1e3
        self.initialized = False
        self.miss = 0
        self.last_plate = -1
        self.n_updates = 0
        self.history = []          # 每帧: dict(state, cov, plate, innov, chi2)
        self._plate_rt = [plate_rt(k, length, width) for k in range(4)]
        self._plate_off = [plate_yaw_offset(k) for k in range(4)]

    # ---------- 基本量 ----------
    @property
    def yaw(self):
        return self.x[6]

    @property
    def pv(self):
        return self.x[0:3]

    def plate_state(self, k):
        """板 k 的预测世界系中心与法线偏航。"""
        R_vp, p_v = self._plate_rt[k]
        yaw = self.x[6]
        R = _rotz(yaw)
        return self.x[0:3] + R @ p_v, yaw + self._plate_off[k]

    def visible_plate(self, cam_pos=np.zeros(3)):
        """按"法线朝向相机"选择当前可见板（整车模型的匹配先验）。"""
        best, best_dot = 0, -2.0
        for k in range(4):
            pk, yawk = self.plate_state(k)
            d = cam_pos - pk
            nd = np.linalg.norm(d)
            if nd < 1e-6:
                continue
            n = _rotz(self.x[6]) @ PLATE_NORMALS[k]
            dot = float(n @ d / nd)
            if dot > best_dot:
                best, best_dot = k, dot
        return best, best_dot

    # ---------- 预测 ----------
    def predict(self, dt):
        if not self.initialized:
            return
        if dt <= 1e-6:
            return
        x = self.x
        F = np.eye(self.N)
        F[0:3, 3:6] = np.eye(3) * dt
        F[6, 7] = dt
        x[0:3] += x[3:6] * dt
        if self.p.ground_constraint:
            F[2, 5] = 0.0
            x[5] = 0.0
        x[6] = x[6] + x[7] * dt
        x[6] = wrap_angle(x[6])
        qa = self.p.q_acc
        qal = self.p.q_alpha
        Q = np.zeros((self.N, self.N))
        Q[0:3, 0:3] = np.eye(3) * (dt ** 3) / 3.0 * qa
        Q[0:3, 3:6] = np.eye(3) * (dt ** 2) / 2.0 * qa
        Q[3:6, 0:3] = np.eye(3) * (dt ** 2) / 2.0 * qa
        Q[3:6, 3:6] = np.eye(3) * dt * qa
        Q[6, 6] = (dt ** 3) / 3.0 * qal
        Q[6, 7] = (dt ** 2) / 2.0 * qal
        Q[7, 6] = (dt ** 2) / 2.0 * qal
        Q[7, 7] = dt * qal
        if self.p.ground_constraint:
            # 地面约束：垂向速度恒 0；垂向位置只保留很小的过程噪声
            Q[5, :] = 0.0
            Q[:, 5] = 0.0
            Q[2, 2] = 0.01 ** 2
            self.P[5, :] = 0.0
            self.P[:, 5] = 0.0
            self.x[5] = 0.0
        self.P = F @ self.P @ F.T + Q
        self.miss += 1

    # ---------- 观测 ----------
    def predict_obs(self, k):
        pk, yawk = self.plate_state(k)
        H = np.zeros((4, self.N))
        H[0:3, 0:3] = np.eye(3)
        R_vp, p_v = self._plate_rt[k]
        d = np.array([-p_v[1], p_v[0], 0.0])
        H[0:3, 6] = _rotz(self.x[6]) @ d * getattr(self.p, "pos_yaw_coupling", 1.0)
        H[3, 6] = 1.0
        return np.concatenate([pk, [wrap_angle(yawk)]]), H

    def measurement_cov(self, k, det_world, view_dir_w, depth, cos_theta):
        """位置噪声沿视线方向放大；偏航噪声随板面倾斜放大。"""
        p = self.p
        sig_lat = p.r_pos_lat + p.r_pos_range * depth
        sig_dep = p.r_pos_depth + p.r_pos_range * depth * 2.0
        d = view_dir_w / max(np.linalg.norm(view_dir_w), 1e-9)
        R3 = sig_lat ** 2 * (np.eye(3) - np.outer(d, d)) + sig_dep ** 2 * np.outer(d, d)
        ct = max(cos_theta, 0.25)
        sig_yaw = math.radians(p.r_yaw_base) + math.radians(p.r_yaw_foreshort) * (1.0 / ct ** 2 - 1.0)
        if self.n_updates > 0 and k != self.last_plate:
            sig_yaw *= p.switch_inflate
        R = np.zeros((4, 4))
        R[0:3, 0:3] = R3
        R[3, 3] = sig_yaw ** 2
        return R

    # ---------- 更新 ----------
    def update_plate(self, k, z, R, gate=True):
        zhat, H = self.predict_obs(k)
        r = np.asarray(z, dtype=float) - zhat
        r[3] = wrap_angle(r[3])
        S = H @ self.P @ H.T + R
        try:
            Sinv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return None
        d2 = float(r @ Sinv @ r)
        if gate and d2 > self.p.gate_chi2:
            return {"accepted": False, "chi2": d2, "plate": k, "residual": r}
        prev = self.last_plate
        is_switch = self.n_updates > 0 and k != prev
        # 位置修正限幅（跳变保护）
        K = self.P @ H.T @ Sinv
        dx = K @ r
        if np.linalg.norm(dx[0:3]) > self.p.max_pos_step:
            dx[0:3] *= self.p.max_pos_step / np.linalg.norm(dx[0:3])
        self.x = self.x + dx
        self.x[6] = wrap_angle(self.x[6])
        IKH = np.eye(self.N) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T   # Joseph 形式
        self.last_plate = k
        self.n_updates += 1
        self.miss = 0
        return {"accepted": True, "chi2": d2, "plate": k, "residual": r,
                "is_switch": is_switch, "R": R}

    def initialize(self, k, z_pos, z_yaw):
        self.x = np.zeros(self.N)
        self.x[0:3] = z_pos
        self.x[6] = wrap_angle(z_yaw - self._plate_off[k])
        self.P = np.diag([0.3 ** 2] * 3 + [1.0 ** 2] * 3 + [math.radians(15) ** 2, math.radians(30) ** 2])
        # 位置初值来自观测, 但扣掉板相对车心的偏置
        R_vp, p_v = self._plate_rt[k]
        self.x[0:3] -= _rotz(self.x[6]) @ p_v
        self.initialized = True
        self.last_plate = k
        self.miss = 0
        self.n_updates = 1

    # ---------- 丢失恢复 ----------
    def lost(self):
        return self.miss > self.p.max_miss
