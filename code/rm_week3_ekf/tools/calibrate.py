"""标定相机-IMU 外参并落盘 configs/extrinsic.json。

用法: python tools/calibrate.py --dets assets/detections.json --out configs/extrinsic.json
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "src"))

from rm_ekf.calib import (estimate_R_bc, load_extrinsic, plate_up_observations,  # noqa: E402
                          save_extrinsic)
from rm_ekf.imu import ImuStream                                              # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dets", default="assets/detections.json")
    ap.add_argument("--imu", default="2026-05-20_23-57-31.txt")
    ap.add_argument("--offset", type=float, default=-0.009)
    ap.add_argument("--out", default=os.path.join(HERE, "configs/extrinsic.json"))
    args = ap.parse_args()
    with open(args.dets, encoding="utf-8") as f:
        dets = json.load(f)
    imu = ImuStream(args.imu, time_offset_s=args.offset)
    for src in ("body_to_world", "world_to_body"):
        uc, zb, w = plate_up_observations(dets, imu, src=src)
        R, info = estimate_R_bc(uc, zb, w)
        print("[%s] n=%d rms=%.2f deg  max=%.2f deg  euler=%s"
              % (src, info["n_samples"], info["up_residual_rms_deg"],
                 info["up_residual_max_deg"],
                 ["%.1f" % v for v in info["euler_zyx_deg"]]))
        if src == "body_to_world":
            save_extrinsic(args.out, R, info)
            print("saved ->", args.out)
            print("R_bc =\n", np.round(R, 4))


if __name__ == "__main__":
    main()
