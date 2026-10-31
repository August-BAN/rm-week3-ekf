"""渲染：四板重投影 + 实时状态 HUD，输出标注视频。"""
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "src"))

from rm_ekf.armor import PLATE_NAMES                  # noqa: E402
from rm_ekf.config import Config                      # noqa: E402
from rm_ekf.render import draw_hud, draw_vehicle      # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(HERE, "configs/default.json"))
    ap.add_argument("--track", default="assets/track.json")
    ap.add_argument("--video", default="2026-05-20_23-57-31.avi")
    ap.add_argument("--out", default="assets/track_vis.avi")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--end", type=int, default=-1)
    ap.add_argument("--every", type=int, default=1)
    args = ap.parse_args()

    cfg = Config(args.config)
    with open(args.track, encoding="utf-8") as f:
        log = json.load(f)
    frames = {r["frame"]: r for r in log["frames"]}
    cap = cv2.VideoCapture(args.video)
    fps = cap.get(cv2.CAP_PROP_FPS)
    writer = None
    idx = 0
    n_out = 0
    while True:
        ok, img = cap.read()
        if not ok:
            break
        if idx < args.start:
            idx += 1
            continue
        if args.end >= 0 and idx > args.end:
            break
        if idx % args.every:
            idx += 1
            continue
        rec = frames.get(idx)
        if rec is not None:
            R_gc = np.array(rec["R_gc"]).reshape(3, 3)
            st = rec["state"]
            std = rec["std"]
            if rec.get("initialized"):
                draw_vehicle(img, np.array(st[0:3]), st[6], log["length"], log["width"],
                             R_gc, cfg.K, cfg.dist, cfg.armor_w, cfg.armor_h)
            p, v, yaw = st[0:3], st[3:6], st[6]
            dist = float(np.linalg.norm(p))
            lines = [
                "t = %5.2f s   frame %3d" % (rec["t"], idx),
                "pos  x=%6.2f y=%6.2f z=%6.2f m" % (p[0], p[1], p[2]),
                "vel  vx=%5.2f vy=%5.2f vz=%5.2f m/s" % (v[0], v[1], v[2]),
                "yaw=%7.1f deg   yawrate=%7.1f deg/s" % (math.degrees(yaw), math.degrees(st[7])),
                "range=%5.2f m   plate=%s   miss=%d" %
                (dist, PLATE_NAMES[rec.get("plate", 0)].upper() if rec.get("initialized") else "-",
                 rec.get("miss", 0)),
                "sigma: p %.2f/%.2f/%.2f m  yaw %.1f deg" %
                (std[0], std[1], std[2], math.degrees(std[6])),
            ]
            draw_hud(img, lines)
        if writer is None:
            h, w = img.shape[:2]
            writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"MJPG"), fps / args.every, (w, h))
        writer.write(img)
        n_out += 1
        idx += 1
    cap.release()
    if writer is not None:
        writer.release()
    print("wrote", args.out, n_out, "frames")


if __name__ == "__main__":
    main()
