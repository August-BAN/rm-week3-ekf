"""从标注视频导出预览帧 / 720p MP4（报告与快速查看用）。"""
import argparse
import os

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", default="assets/track_vis.avi")
    ap.add_argument("--outdir", default="assets/preview")
    ap.add_argument("--frames", default="0,150,300,450")
    ap.add_argument("--mp4", default="", help="若给出路径, 另存 960x720 MP4")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    want = [int(s) for s in args.frames.split(",") if s.strip()]
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit("cannot open " + args.video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    writer = None
    if args.mp4:
        w = cv2.VideoWriter(args.mp4, cv2.VideoWriter_fourcc(*"mp4v"), fps, (960, 720))
        writer = w if w.isOpened() else None
        if writer is None:
            print("warning: mp4v writer unavailable, skip mp4")
    i = 0
    while True:
        ok, f = cap.read()
        if not ok:
            break
        if i in want:
            p = os.path.join(args.outdir, "frame_%04d.png" % i)
            cv2.imwrite(p, f)
            print("saved", p)
        if writer is not None:
            writer.write(cv2.resize(f, (960, 720), interpolation=cv2.INTER_AREA))
        i += 1
    cap.release()
    if writer is not None:
        writer.release()
        print("saved", args.mp4, "%.1f MB" % (os.path.getsize(args.mp4) / 1e6))


if __name__ == "__main__":
    main()
