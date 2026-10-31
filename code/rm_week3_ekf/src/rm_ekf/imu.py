"""IMU 四元数读取、时间同步（采集时间 = 收到时间 + offset）与姿态插值。"""
import numpy as np

from .geometry import quat_to_R, slerp


class ImuStream:
    def __init__(self, path, time_offset_s=-0.009, quat_order="wxyz"):
        data = np.loadtxt(path)
        self.t = data[:, 0].astype(np.float64)
        self.q = data[:, 1:5].astype(np.float64)
        n = np.linalg.norm(self.q, axis=1, keepdims=True)
        self.q = self.q / np.maximum(n, 1e-12)
        self.offset = float(time_offset_s)
        self.order = quat_order
        if self.order == "xyzw":
            self.q = self.q[:, [3, 0, 1, 2]]

    def quat_at(self, t_query):
        """查询 t_query 时刻的姿态四元数（slerp）。"""
        t = np.asarray(t_query, dtype=np.float64)
        scalar = t.ndim == 0
        tt = np.atleast_1d(t)
        idx = np.searchsorted(self.t, tt)
        idx = np.clip(idx, 1, len(self.t) - 1)
        t0 = self.t[idx - 1]
        t1 = self.t[idx]
        w = np.clip((tt - t0) / np.maximum(t1 - t0, 1e-12), 0.0, 1.0)
        out = np.zeros((len(tt), 4))
        for i in range(len(tt)):
            out[i] = slerp(self.q[idx[i] - 1], self.q[idx[i]], w[i])
        return out[0] if scalar else out

    def R_at(self, t_query, src="body_to_world"):
        q = self.quat_at(t_query)
        R = quat_to_R(q)
        return R if src == "body_to_world" else R.T

    def frame_time(self, t_img):
        """图像时间戳 -> 需要查询的 IMU 时间戳。"""
        return np.asarray(t_img, dtype=np.float64) + self.offset

    # ---- 按帧号对齐（本数据集 IMU 样本与视频帧一一对应）----
    def time_of_frame(self, frame_index):
        """第 i 帧对应的 IMU 查询时刻：该帧采样时刻的 offset 之前。

        IMU 以 24 Hz 内录 531 个四元数，视频为同一段 531 帧（30 fps 容器，
        时间轴被压缩到 1/1.25）。因此对应关系是"帧号 <-> 样本号"一一对应，
        而不是按视频播放时间对齐；这一点由图像竖直方向与 IMU pitch 的
        相关系数 0.995（τ = t0 + 1.25 * t_video）实测确认。
        """
        i = int(np.clip(frame_index, 0, len(self.t) - 1))
        return self.t[i] - 0.009

    def R_of_frame(self, frame_index, src="body_to_world"):
        return self.R_at(self.time_of_frame(frame_index), src=src)
