"""配置加载：JSON + 默认值，便于命令行覆盖。"""
import json
import os

import numpy as np

DEFAULT = {
    "camera": {
        "width": 1440, "height": 1080,
        "fx": 2362.2795952460974, "fy": 2336.1916035376717,
        "cx": 679.17513921636282, "cy": 529.06858905837635,
        "dist": [-0.098561592108101559, 0.11173066531932681,
                 -0.0038352945867108721, -0.0020421443354436304, 0.0],
        "reproj_error_px": 0.1448,
    },
    "armor": {"width": 0.27, "height": 0.103,
              "vehicle_length": 1.10, "vehicle_width": 0.75},
    "imu": {"file": "2026-05-20_23-57-31.txt", "time_offset_s": -0.009,
            "quat_order": "wxyz"},
    "detector": {},
    "solver": {"max_reproj": 8.0, "min_z": 0.05},
    "ekf": {},
}


def _deep_update(base, extra):
    for k, v in extra.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_update(base[k], v)
        else:
            base[k] = v
    return base


class Config:
    def __init__(self, path=None):
        raw = json.loads(json.dumps(DEFAULT))
        if path and os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                _deep_update(raw, json.load(f))
        self.raw = raw

    # --- 相机 ---
    @property
    def K(self):
        c = self.raw["camera"]
        return np.array([[c["fx"], 0.0, c["cx"]],
                         [0.0, c["fy"], c["cy"]],
                         [0.0, 0.0, 1.0]], dtype=np.float64)

    @property
    def dist(self):
        return np.array(self.raw["camera"]["dist"], dtype=np.float64)

    @property
    def image_size(self):
        c = self.raw["camera"]
        return int(c["width"]), int(c["height"])

    # --- 目标 ---
    @property
    def armor_w(self):
        return float(self.raw["armor"]["width"])

    @property
    def armor_h(self):
        return float(self.raw["armor"]["height"])

    @property
    def vehicle_length(self):
        return float(self.raw["armor"]["vehicle_length"])

    @property
    def vehicle_width(self):
        return float(self.raw["armor"]["vehicle_width"])

    @property
    def imu_cfg(self):
        return self.raw["imu"]

    def get(self, section, key, default=None):
        return self.raw.get(section, {}).get(key, default)
