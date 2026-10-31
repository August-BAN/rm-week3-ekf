"""一键跑完全流程：检测 -> 外参标定 -> 整车 EKF -> 状态曲线 -> 标注视频。

用法（在作业根目录，含 avi/txt 的目录下运行）:
    python code/rm_week3_ekf/run_all.py
    python code/rm_week3_ekf/run_all.py --skip-calib      # 复用已有外参
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(HERE, "tools")
PY = sys.executable


def run(script, *args):
    cmd = [PY, os.path.join(TOOLS, script)] + list(args)
    print("\n>>>", " ".join(os.path.basename(c) for c in cmd), flush=True)
    t0 = time.time()
    r = subprocess.run(cmd, cwd=os.getcwd())
    print("<<< exit=%d  %.1fs" % (r.returncode, time.time() - t0), flush=True)
    if r.returncode != 0:
        raise SystemExit("step failed: " + script)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", default="2026-05-20_23-57-31.avi")
    ap.add_argument("--imu", default="2026-05-20_23-57-31.txt")
    ap.add_argument("--config", default=os.path.join(HERE, "configs/default.json"))
    ap.add_argument("--extrinsic", default=os.path.join(HERE, "configs/extrinsic.json"))
    ap.add_argument("--assets", default="assets")
    ap.add_argument("--skip-det", action="store_true")
    ap.add_argument("--skip-calib", action="store_true")
    ap.add_argument("--no-video", action="store_true")
    args = ap.parse_args()

    dets = os.path.join(args.assets, "detections.json")
    track = os.path.join(args.assets, "track.json")
    os.makedirs(args.assets, exist_ok=True)

    if not args.skip_det:
        run("dump_detections.py", "--config", args.config, "--video", args.video,
            "--out", dets)
    if not args.skip_calib:
        run("calibrate.py", "--dets", dets, "--imu", args.imu, "--out", args.extrinsic)
    run("run_tracker.py", "--config", args.config, "--extrinsic", args.extrinsic,
        "--dets", dets, "--imu", args.imu, "--out", track)
    run("plot_states.py", "--track", track, "--outdir", os.path.join(args.assets, "curves"))
    if not args.no_video:
        run("render_video.py", "--config", args.config, "--track", track,
            "--video", args.video, "--out", os.path.join(args.assets, "track_vis.avi"))
        run("make_preview.py", "--video", os.path.join(args.assets, "track_vis.avi"),
            "--outdir", os.path.join(args.assets, "preview"),
            "--mp4", os.path.join(args.assets, "track_vis_720p.mp4"))
    print("\nAll done. 输出在", os.path.abspath(args.assets))


if __name__ == "__main__":
    main()
