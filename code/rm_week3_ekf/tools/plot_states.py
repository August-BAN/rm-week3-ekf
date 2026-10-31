"""生成状态曲线图（8 个状态量 + 协方差带 + 一致性指标）。"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "src"))

from rm_ekf.plotting import PALETTE, Figure   # noqa: E402


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def series(log):
    t, st, sd = [], [], []
    for r in log["frames"]:
        t.append(r["t"])
        st.append(r["state"])
        sd.append(r["std"])
    return np.array(t), np.array(st), np.array(sd)


def fmt(v, n=2):
    return ("%%.%df" % n) % v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", default="assets/track.json")
    ap.add_argument("--outdir", default="assets/curves")
    args = ap.parse_args()
    log = load(args.track)
    t, st, sd = series(log)
    os.makedirs(args.outdir, exist_ok=True)

    # ---- 图 1：位置 ----
    fig = Figure(1500, 760)
    names = ["x", "y", "z"]
    units = "m"
    rects = [(0.07, 0.08, 0.97, 0.36), (0.07, 0.40, 0.97, 0.68), (0.07, 0.72, 0.97, 1.00)]
    for i, (nm, rc) in enumerate(zip(names, rects)):
        ax = fig.axes(rc, title="位置 %s 估计（含 ±2σ 带）" % nm, xlabel="时间 t (s)",
                      ylabel="%s / %s" % (nm, units))
        ax.band(t, st[:, i] - 2 * sd[:, i], st[:, i] + 2 * sd[:, i], (80, 120, 220),
                alpha=40, label="±2σ")
        ax.plot(t, st[:, i], color=PALETTE[i], label="EKF 估计", width=2.2)
        ax.legend(pos="tr")
        ax.draw()
    fig.save(os.path.join(args.outdir, "01_position.png"))

    # ---- 图 2：速度 ----
    fig = Figure(1500, 900)
    for i, nm in enumerate(["vx", "vy", "vz"]):
        ax = fig.axes((0.07, 0.06 + i * 0.30, 0.97, 0.30 + i * 0.30),
                      title="速度 %s 估计（含 ±2σ 带）" % nm, xlabel="时间 t (s)", ylabel="m/s")
        ax.band(t, st[:, 3 + i] - 2 * sd[:, 3 + i], st[:, 3 + i] + 2 * sd[:, 3 + i],
                (80, 120, 220), alpha=40, label="±2σ")
        ax.plot(t, st[:, 3 + i], color=PALETTE[1 + i], label="EKF 估计", width=2.2)
        ax.hline(0.0, color=(150, 150, 150), width=0.8)
        ax.legend(pos="tr")
        ax.draw()
    fig.save(os.path.join(args.outdir, "02_velocity.png"))

    # ---- 图 3：偏航角与角速度 ----
    yaw_unwrap = np.unwrap(st[:, 6])
    fig = Figure(1500, 820)
    ax = fig.axes((0.07, 0.08, 0.97, 0.46), title="偏航角 psi（解缠后，deg）",
                  xlabel="时间 t (s)", ylabel="deg")
    ax.plot(t, np.degrees(yaw_unwrap), color=PALETTE[4], label="EKF 估计", width=2.2)
    ax.plot(t, np.degrees(st[:, 6]), color=PALETTE[0], label="wrap 到 ±180°", width=1.4)
    ax.legend()
    ax.draw()
    ax = fig.axes((0.07, 0.56, 0.97, 0.94), title="偏航角速度 dpsi/dt（deg/s）",
                  xlabel="时间 t (s)", ylabel="deg/s")
    ax.plot(t, np.degrees(st[:, 7]), color=PALETTE[2], label="EKF 估计", width=2.0)
    ax.plot(t, np.gradient(np.degrees(yaw_unwrap), t), color=(150, 150, 150),
            label="对解缠角差分", width=1.2)
    ax.hline(0.0, color=(170, 170, 170), width=0.8)
    ax.legend()
    ax.draw()
    fig.save(os.path.join(args.outdir, "03_yaw.png"))

    # ---- 图 4：新息与卡方检验 ----
    fr, dx, dy, dz, dyaw, chi2, acc = [], [], [], [], [], [], []
    for r in log["frames"]:
        for u in r.get("updates", []):
            if "residual" not in u:
                continue
            fr.append(r["t"])
            res = u["residual"]
            dx.append(res[0])
            dy.append(res[1])
            dz.append(res[2])
            dyaw.append(math.degrees(res[3]))
            chi2.append(u["chi2"])
            acc.append(1.0 if u.get("accepted", True) else 0.0)
    fig = Figure(1500, 900)
    ax = fig.axes((0.07, 0.06, 0.97, 0.30), title="观测新息（位置，m）", xlabel="时间 t (s)", ylabel="m")
    ax.plot(fr, dx, color=PALETTE[0], label="Δx", width=1.4)
    ax.plot(fr, dy, color=PALETTE[1], label="Δy", width=1.4)
    ax.plot(fr, dz, color=PALETTE[2], label="Δz", width=1.4)
    ax.legend(pos="tr")
    ax.draw()
    ax = fig.axes((0.07, 0.38, 0.97, 0.62), title="观测新息（偏航，deg）", xlabel="时间 t (s)", ylabel="deg")
    ax.plot(fr, dyaw, color=PALETTE[4], label="Δψ", width=1.4)
    ax.legend(pos="tr")
    ax.draw()
    ax = fig.axes((0.07, 0.70, 0.97, 0.94), title="马氏距离 d2 与卡方门限", xlabel="时间 t (s)", ylabel="d2")
    thr = float(log.get("gate_chi2", 80.0)) if "gate_chi2" in log else 80.0
    ax.hline(thr, color=(200, 60, 60), width=1.2, label="门限 %.1f" % thr)
    ax.plot(fr, chi2, color=(90, 90, 90), label="d2", width=1.4)
    ax.legend(pos="tr")
    ax.draw()
    fig.save(os.path.join(args.outdir, "04_innovation.png"))

    # ---- 图 5：重投影误差 ----
    rf, rp, dp = [], [], []
    for r in log["frames"]:
        for u in r.get("updates", []):
            if "reproj_px" in u and u.get("accepted", True):
                rf.append(r["t"])
                rp.append(u["reproj_px"])
                dp.append(u["depth"])
    fig = Figure(1500, 760)
    ax = fig.axes((0.07, 0.08, 0.97, 0.48),
                  title="观测板重投影误差（状态预测 vs 检测角点，px）", xlabel="时间 t (s)", ylabel="px")
    ax.plot(rf, rp, color=PALETTE[0], label="每帧", width=1.2)
    med = float(np.median(rp))
    ax.hline(med, color=PALETTE[1], width=1.2, label="中位数 %.1f px" % med)
    ax.legend(pos="tr")
    ax.draw()
    ax = fig.axes((0.07, 0.58, 0.97, 0.94), title="观测板距离（m）", xlabel="时间 t (s)", ylabel="m")
    ax.plot(rf, dp, color=PALETTE[2], label="PnP 深度", width=1.2)
    ax.legend(pos="tr")
    ax.draw()
    fig.save(os.path.join(args.outdir, "05_reprojection.png"))

    # ---- 图 6：俯视轨迹 ----
    fig = Figure(900, 900)
    ax = fig.axes((0.10, 0.08, 0.96, 0.92),
                  title="水平面轨迹（世界系，相机在原点）", xlabel="x (m)", ylabel="y (m)")
    ax.plot(st[:, 0], st[:, 1], color=PALETTE[1], label="EKF 轨迹", width=2.2)
    ax.hline(0.0, color=(220, 220, 220), width=1.0)
    ax.vline(0.0, color=(220, 220, 220), width=1.0)
    ax.legend()
    ax.draw()
    fig.save(os.path.join(args.outdir, "06_trajectory.png"))

    print("saved curves ->", args.outdir)
    relative_figure(log, args.outdir)
    print("yaw span %.1f deg, vyaw mean %.1f deg/s"
          % (np.degrees(yaw_unwrap.max() - yaw_unwrap.min()), np.degrees(np.mean(st[:, 7]))))


def relative_figure(log, outdir):
    """补充：相对观测者的量（距离/方位/相对偏航），物理上最可解释。"""
    import numpy as np
    tf, rng, brg, zz, dep = [], [], [], [], []
    for r in log["frames"]:
        if not r.get("initialized"):
            continue
        st = r["state"]
        p = np.array(st[0:3])
        tf.append(r["t"])
        rng.append(float(np.linalg.norm(p)))
        brg.append(math.degrees(math.atan2(p[1], p[0])))
        zz.append(float(p[2]))
        ds = r.get("dets_world") or []
        dep.append(ds[0]["depth"] if ds else float("nan"))
    fig = Figure(1500, 820)
    ax = fig.axes((0.07, 0.07, 0.97, 0.34), title="目标到相机的距离 |p| 与 PnP 深度",
                  xlabel="时间 t (s)", ylabel="m")
    ax.plot(tf, rng, color=PALETTE[1], label="状态 |p|", width=2.0)
    ax.plot(tf, dep, color=PALETTE[3], label="PnP 深度", width=1.2)
    ax.legend(pos="tr")
    ax.draw()
    ax = fig.axes((0.07, 0.42, 0.97, 0.66), title="目标方位角（世界系，deg）",
                  xlabel="时间 t (s)", ylabel="deg")
    ax.plot(tf, brg, color=PALETTE[4], label="bearing", width=1.6)
    ax.legend(pos="tr")
    ax.draw()
    ax = fig.axes((0.07, 0.72, 0.97, 0.96), title="目标高度 z（世界系，m）",
                  xlabel="时间 t (s)", ylabel="m")
    ax.plot(tf, zz, color=PALETTE[2], label="z", width=1.6)
    ax.legend(pos="tr")
    ax.draw()
    fig.save(os.path.join(outdir, "07_relative.png"))


if __name__ == "__main__":
    main()
