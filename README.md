# 27 赛季算法组视觉侧 第三周作业

**装甲板检测 → PnP 位姿解算 → 整车（四块板）EKF → 状态曲线 + 四板重投影视频**

> 作业报告：[`视觉第三周作业_季弋塬.md`](视觉第三周作业_季弋塬.md)

## 这一版做了什么

| 环节 | 实现 | 关键结论 |
|---|---|---|
| 检测 + PnP | `code/rm_week3_ekf/src/rm_ekf/detector.py`、`solver.py` | 531 帧检出 583 块板，平 12.2 ms/帧；PnP 100% 成功，残差均值 1.49 px |
| 时间同步 | `imu.py`、`tools/calibrate.py` | IMU 与视频是**帧号一一对应**（24 Hz ↔ 30 fps），不是按播放时间对齐；相关性 0.995 |
| 相机-IMU 外参 | `calib.py` | 用"板 y 轴 = 重力反方向"做 Wahba 配准，残差 RMS 7.65° |
| 整车 EKF | `ekf.py`、`pipeline.py` | 8 维 `[x,y,z,vx,vy,vz,ψ,ψ̇]`，自适应观测噪声 + 卡方门限 + 换板迟滞 |
| 可视化 | `render.py`、`plotting.py` | 四板重投影（正/背面区分）+ 实时状态 HUD；7 张状态曲线（含 ±2σ） |

**精度**：状态预测板与检测角点的重投影误差中位数 **12.8 px**（≈1.5 cm @ 2.7 m），77% 帧优于 20 px。

## 目录

```
视觉第三周作业_季弋塬.md      作业报告（原理/方程/匹配/跳变/结果/思考）
code/rm_week3_ekf/            工程实现
  ├── run_all.py              一键：检测→标定→EKF→曲线→视频
  ├── configs/                全部参数 + 标定出的外参
  ├── src/rm_ekf/             geometry/armor/detector/solver/imu/calib/ekf/pipeline/render/plotting
  └── tools/                  命令行脚本
assets/
  ├── curves/                 7 张状态曲线 PNG
  ├── preview/                4 张标注帧 PNG
  ├── track_vis_720p.mp4      四板重投影 + HUD 视频（960×720）
  ├── detections.json         逐帧检测与 PnP 结果
  └── track.json              EKF 逐帧状态、协方差、新息
```

## 快速开始

```bash
git clone https://github.com/August-BAN/rm-week3-ekf.git
cd rm-week3-ekf

# 1) 把作业素材放回根目录（体积大，未入库）
#    2026-05-20_23-57-31.avi
#    2026-05-20_23-57-31.txt

# 2) 装依赖并一键跑完
python -m pip install -r code/rm_week3_ekf/requirements.txt
python code/rm_week3_ekf/run_all.py
```

约 85 s 后得到 `assets/detections.json`、`assets/track.json`、`assets/curves/*.png`、`assets/track_vis.avi`。

## 说明

- **素材与大文件不入库**：原始视频 64 MB、全分辨率标注视频 80 MB 都在 `.gitignore` 里；
  仓库只保留 26.8 MB 的 720p 预览版，方便直接在线看效果。
- 目标车体按长方形建模（1.10 × 0.75 m，对角 0.66 m 由"同帧两板中心距"反解）；
  装甲板是自制件，灯条间距 : 灯条长 ≈ 2.62 : 1，绝对尺度以"板宽 0.27 m"为锚点。
- 观测者（相机）自身平移没有里程计可用，因此状态本质是**目标相对观测者**的估计，
  这一点在报告 §7(2) 有专门讨论。

## 环境

Windows 11 + Python 3.12.10 + numpy 2.5.4 + OpenCV 5.0.0（headless）+ Pillow。
无 matplotlib，曲线图由 `plotting.py` 用 Pillow 自绘（3× 超采样）。
