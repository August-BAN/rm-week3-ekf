# rm-week3-ekf — 装甲板四板整车 EKF

27 赛季算法组视觉侧第三周作业的工程实现：装甲板检测 → PnP 位姿解算 → 整车（四块板）
EKF → 状态曲线与四板重投影视频。

仓库：<https://github.com/August-BAN/rm-week3-ekf>（原 Gitee 版已迁入；素材与大体积视频不入库，
clone 后按下方"运行"把 `.avi/.txt` 放回作业根目录即可复现）。

## 依赖

- Python 3.10+（实测 3.12.10）
- numpy、opencv-python（实测 numpy 2.5.4 / OpenCV 5.0.0，headless 版即可）
- Pillow（仅用于画状态曲线，环境无 matplotlib 时自绘）

```bash
python -m pip install -r requirements.txt
```

## 运行

在**含素材的作业根目录**（`2026-05-20_23-57-31.avi` / `.txt` 所在目录）执行：

```bash
python code/rm_week3_ekf/run_all.py
```

分步执行：

```bash
python code/rm_week3_ekf/tools/dump_detections.py --out assets/detections.json
python code/rm_week3_ekf/tools/calibrate.py        --out code/rm_week3_ekf/configs/extrinsic.json
python code/rm_week3_ekf/tools/run_tracker.py      --out assets/track.json
python code/rm_week3_ekf/tools/plot_states.py      --track assets/track.json --outdir assets/curves
python code/rm_week3_ekf/tools/render_video.py     --track assets/track.json --out assets/track_vis.avi
```

实测耗时（i5 / Windows）：检测 14 s，EKF 1 s，曲线 3 s，渲染 60 s。

## 目录

```
configs/default.json     全部参数（相机/装甲板/车体/检测/EKF）
configs/extrinsic.json   相机-IMU 外参 R_bc（标定产物）
src/rm_ekf/
  geometry.py   SO(3)/四元数/重力对齐系
  armor.py      四板整车模型（板坐标系、板位姿、角点）
  detector.py   灯条分割 + 比值配对 + 打分/贪心/NMS
  solver.py     PnP（IPPE + LM 精修，畸变参与优化）
  imu.py        四元数读取、帧号对齐、slerp
  calib.py      相机-IMU 外参（重力方向 Wahba 配准）
  ekf.py        整车 EKF（8 维状态、自适应观测噪声、卡方门限、Joseph 更新）
  pipeline.py   单帧流程：关联 + 更新 + 日志（含重投影误差）
  render.py     四板重投影 / 车体线框 / HUD
  plotting.py   PIL 自绘折线图（含 ±2σ 带、中文标注）
tools/          命令行脚本（见上）
```

## 坐标系

| 系 | 定义 |
|---|---|
| 相机系 C | OpenCV 光学系：x 右、y 下、z 前，单位 m |
| 世界系 W | 重力对齐，z 轴向上；由 IMU 姿态 + 外参给出，原点/航向取首帧相机位姿 |
| 车体系 V | x 前、y 左、z 上；原点在四块板中心所在平面中心 |
| 板系 P | x 观看者右、y 上、z 外法线，右手系；z=0 为板面 |

目标车体按长方形建模：前后板中心距 `vehicle_length=1.10 m`，左右板中心距
`vehicle_width=0.75 m`（对角距 0.66 m，由视频中同帧两块板中心距反解）。

## 已知限制

- 观测者（相机）自身平移无里程计/加速度计可测，只能作为过程噪声吸收；
  估计结果因此是"目标相对观测者"的状态，绝对轨迹需要底盘里程计才能恢复。
- 视频中的目标板为自制件（灯条间距:灯条长 ≈ 2.62:1，非标准 135:125），
  绝对尺度由"板宽 0.27 m"这一假设锚定，全部距离随该假设线性缩放。
