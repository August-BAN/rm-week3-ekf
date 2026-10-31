"""逐帧检测 + PnP，结果落盘为 JSON 供后续分析/跟踪复用。

用法:
    python tools/dump_detections.py --config configs/default.json --out assets/detections.json
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "src"))

from rm_ekf.config import Config                        # noqa: E402
from rm_ekf.detector import DetectorConfig, detect      # noqa: E402
from rm_ekf.solver import solve_armor_pnp               # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(HERE, "configs/default.json"))
    ap.add_argument("--video", default="2026-05-20_23-57-31.avi")
    ap.add_argument("--out", default="assets/detections.json")
    ap.add_argument("--vis", default="")
    ap.add_argument("--every", type=int, default=1)
    args = ap.parse_args()

    cfg = Config(args.config)
    det_cfg = DetectorConfig(**cfg.raw["detector"])
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit("cannot open " + args.video)
    fps = cap.get(cv2.CAP_PROP_FPS)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frames = []
    writer = None
    t0 = time.time()
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if i % args.every:
            i += 1
            continue
        ts = time.time()
        mask, dets = detect(frame, det_cfg)
        ms = (time.time() - ts) * 1000.0
        rec = {"frame": i, "t": i / fps, "detections": []}
        for d in dets:
            pr = solve_armor_pnp(d.corners, cfg.K, cfg.dist,
                                 cfg.armor_w, cfg.armor_h,
                                 max_reproj=cfg.get("solver", "max_reproj", 8.0))
            wpx = float(np.linalg.norm(d.corners[3] - d.corners[0]))
            rec["detections"].append({
                "corners": d.corners.reshape(-1).tolist(),
                "score": float(d.score),
                "width_px": wpx,
                "ok": bool(pr.ok),
                "reproj": float(pr.reproj) if np.isfinite(pr.reproj) else None,
                "tvec": pr.t.reshape(-1).tolist() if pr.t is not None else None,
                "rvec": pr.rvec.reshape(-1).tolist() if pr.rvec is not None else None,
                "R": pr.R.reshape(-1).tolist() if pr.R is not None else None,
                "reason": pr.reason,
            })
            if args.vis:
                c = d.corners.astype(int)
                cv2.polylines(frame, [c], True, (0, 255, 0), 2)
                txt = "%d%%" % int(100 * d.score)
                cv2.putText(frame, txt, tuple(c[0] + np.array([0, -6])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        rec["ms"] = ms
        frames.append(rec)
        if args.vis:
            if writer is None:
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(args.vis, cv2.VideoWriter_fourcc(*"MJPG"), fps, (w, h))
            writer.write(frame)
        i += 1
        if i % 100 == 0:
            print("frame", i, "elapsed %.1fs" % (time.time() - t0), flush=True)
    cap.release()
    if writer is not None:
        writer.release()
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump({"fps": fps, "n_frames": n, "frames": frames}, f)
    ok_cnt = sum(1 for r in frames for d in r["detections"] if d["ok"])
    tot = sum(len(r["detections"]) for r in frames)
    ms = np.array([r["ms"] for r in frames])
    print("frames=%d dets=%d solved=%d  avg %.1f ms/frame  total %.1fs"
          % (len(frames), tot, ok_cnt, ms.mean(), time.time() - t0))


if __name__ == "__main__":
    main()
