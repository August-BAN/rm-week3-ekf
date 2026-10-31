"""跑整车 EKF：检测 -> 世界系观测 -> 跟踪，输出日志 JSON。"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRAME = "world"
sys.path.insert(0, os.path.join(HERE, "src"))

from rm_ekf.calib import load_extrinsic            # noqa: E402
from rm_ekf.config import Config                  # noqa: E402
from rm_ekf.ekf import EKFParams, VehicleEKF      # noqa: E402
from rm_ekf.geometry import gravity_frame         # noqa: E402
from rm_ekf.imu import ImuStream                  # noqa: E402
from rm_ekf.pipeline import run_frame             # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(HERE, "configs/default.json"))
    ap.add_argument("--extrinsic", default=os.path.join(HERE, "configs/extrinsic.json"))
    ap.add_argument("--dets", default="assets/detections.json")
    ap.add_argument("--imu", default="2026-05-20_23-57-31.txt")
    ap.add_argument("--out", default="assets/track.json")
    args = ap.parse_args()

    cfg = Config(args.config)
    R_bc, ext_info = load_extrinsic(args.extrinsic)
    imu = ImuStream(args.imu, time_offset_s=cfg.imu_cfg["time_offset_s"])
    with open(args.dets, encoding="utf-8") as f:
        dets = json.load(f)
    global FRAME
    FRAME = cfg.raw.get("tracker", {}).get("frame", "world")
    params = EKFParams(**cfg.raw["ekf"])
    ekf = VehicleEKF(cfg.vehicle_length, cfg.vehicle_width, params)

    log = {"fps": dets["fps"], "n_frames": dets["n_frames"],
           "extrinsic": ext_info, "length": cfg.vehicle_length,
           "width": cfg.vehicle_width, "frames": []}
    prev_t = None
    for rec in dets["frames"]:
        t_img = rec["t"]
        R_wb = imu.R_of_frame(rec["frame"])
        R_wc = R_wb @ R_bc
        up_c = R_wc.T @ np.array([0.0, 0.0, 1.0])
        R_gc = gravity_frame(up_c)
        if FRAME == "world":
            R_use = R_wc
        else:
            R_use = R_gc
        dt = 0.0 if prev_t is None else (t_img - prev_t)
        prev_t = t_img
        out = {"frame": rec["frame"], "t": t_img,
               "R_gc": R_use.reshape(-1).tolist(),
               "up_c": up_c.tolist()}
        run_frame(ekf, [d for d in rec["detections"] if d["ok"]], R_use,
                  cfg, dt, out, params)
        log["frames"].append(out)

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(log, f)

    # 统计
    n_init = sum(1 for f in log["frames"] if f.get("init"))
    n_upd = sum(len([u for u in f.get("updates", []) if u.get("accepted", True)]) for f in log["frames"])
    n_rej = sum(len([u for u in f.get("updates", []) if not u.get("accepted", True)]) for f in log["frames"])
    miss = [f.get("miss", 0) for f in log["frames"]]
    print("frames=%d init=%d updates=%d rejected=%d  max_miss=%d  avg_miss=%.2f"
          % (len(log["frames"]), n_init, n_upd, n_rej, max(miss), np.mean(miss)))
    st = np.array([f["state"] for f in log["frames"]])
    print("state mean", np.round(st.mean(axis=0), 2))
    print("state std ", np.round(st.std(axis=0), 2))


if __name__ == "__main__":
    main()
