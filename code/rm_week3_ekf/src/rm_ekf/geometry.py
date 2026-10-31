"""SO(3)/SE(3) 与四元数工具。全部使用右手系、列向量、弧度。"""
import math

import numpy as np


def skew(v):
    v = np.asarray(v, dtype=np.float64).reshape(3)
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def quat_to_R(q, order="wxyz"):
    """四元数 -> 旋转矩阵。默认 q=(w,x,y,z)，返回 body->world 的 R。"""
    q = np.asarray(q, dtype=np.float64).reshape(4)
    if order == "xyzw":
        q = np.array([q[3], q[0], q[1], q[2]])
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = q / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def R_to_quat(R):
    """旋转矩阵 -> 四元数 (w,x,y,z)。"""
    R = np.asarray(R, dtype=np.float64)
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def slerp(q0, q1, t):
    """四元数球面线性插值 (wxyz)。"""
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1 = -q1
        d = -d
    d = min(d, 1.0)
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th0 = math.acos(d)
    th = th0 * t
    s0 = math.sin(th0 - th) / math.sin(th0)
    s1 = math.sin(th) / math.sin(th0)
    q = s0 * q0 + s1 * q1
    return q / np.linalg.norm(q)


def so3_log(R):
    """R -> 旋转向量。"""
    R = np.asarray(R, dtype=np.float64)
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) * 0.5))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    if abs(math.pi - th) < 1e-6:
        # 近 pi：从对角线取轴
        A = (R + np.eye(3)) * 0.5
        v = np.sqrt(np.maximum(np.diag(A), 0.0))
        i = int(np.argmax(v))
        axis = np.zeros(3)
        axis[i] = v[i]
        for j in range(3):
            if j != i:
                axis[j] = A[i, j] / max(v[i], 1e-9)
        axis /= np.linalg.norm(axis)
        return axis * th
    return th / (2.0 * math.sin(th)) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def so3_exp(w):
    w = np.asarray(w, dtype=np.float64).reshape(3)
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3) + skew(w)
    a = w / th
    K = skew(a)
    return np.eye(3) + math.sin(th) * K + (1.0 - math.cos(th)) * (K @ K)


def wrap_angle(a):
    """把角度归一到 [-pi, pi)。"""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def mean_rotation(Rs, weights=None):
    """旋转集合的弦均值（SVD 投影到 SO(3)）。"""
    Rs = np.asarray(Rs, dtype=np.float64)
    if Rs.ndim != 3:
        raise ValueError("expect (N,3,3)")
    if weights is None:
        M = Rs.sum(axis=0)
    else:
        w = np.asarray(weights, dtype=np.float64).reshape(-1, 1, 1)
        M = (Rs * w).sum(axis=0)
    U, _, Vt = np.linalg.svd(M)
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    return U @ D @ Vt


def euler_zyx(R):
    """R -> (roll, pitch, yaw)，ZYX 欧拉角。"""
    R = np.asarray(R, dtype=np.float64)
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    pitch = math.atan2(-R[2, 0], sy)
    if sy > 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def angle_between_axes(R1, R2):
    """两个旋转的夹角（deg）。"""
    return math.degrees(np.linalg.norm(so3_log(R1.T @ R2)))


def gravity_frame(up_c):
    """由"重力方向在相机系中的表达"构造重力对齐的相机系 G。

    返回 R_gc（相机系 -> G 的旋转），使 p_G = R_gc @ p_C。
    G 的 z 轴 = 真实竖直（重力反方向），x 轴 = 相机右方在水平面的投影，
    y 轴 = 相机光轴在水平面的投影（前）。
    """
    up = np.asarray(up_c, dtype=np.float64).reshape(3)
    up = up / max(np.linalg.norm(up), 1e-9)
    f = np.array([0.0, 0.0, 1.0])          # 相机光轴（相机系）
    fh = f - float(f @ up) * up
    n = np.linalg.norm(fh)
    if n < 1e-6:                            # 光轴与竖直几乎平行，退化
        fh = np.array([1.0, 0.0, 0.0])
        fh = fh - float(fh @ up) * up
        n = np.linalg.norm(fh)
    y_g = fh / max(n, 1e-9)
    z_g = up
    x_g = np.cross(y_g, z_g)
    R_gc = np.vstack([x_g, y_g, z_g])
    return R_gc
