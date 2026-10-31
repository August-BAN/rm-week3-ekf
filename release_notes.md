## 作业素材（.avi / .txt）

仓库为了体积（`.gitignore` 排除），原始素材不放仓库树里，改为挂在本 Release 下。
下载后放到**作业根目录**（与 `README.md`、`code/` 同级），即可一键复现全部结果。

### 附件

| 文件 | 大小 | SHA256 |
|---|---|---|
| `2026-05-20_23-57-31.avi` | 66,975,040 B (66.98 MB) | `864f7d83000885beee606c2dec2025da5bfe6cf6b91350b77d34d33b04716c3f` |
| `2026-05-20_23-57-31.txt` | 49,036 B (0.05 MB) | `950b2695188565207fc606c201ca013125f9e632623fb6a6f10a26a49b6c9c00` |

### 内容说明

- **`.avi`**：1440×1080、531 帧（30 fps 容器，MJPG）的实拍内录视频，画面里是一台编号 `4` 的履带车
  与四块自制装甲板。
- **`.txt`**：同一段时间的 IMU 内录，531 行、每行 5 列 —— `时间戳(s) qw qx qy qz` 的**接收时间戳**。
  与图像对应时取**帧号一一对应**（24 Hz 采样 ↔ 531 帧），再按题目约定往前推 9 ms。

### 复现步骤

```bash
git clone https://github.com/August-BAN/rm-week3-ekf.git
cd rm-week3-ekf
# 把本 Release 的两个附件下载到当前目录
python -m pip install -r code/rm_week3_ekf/requirements.txt
python code/rm_week3_ekf/run_all.py
```

约 85 s 后得到 `assets/detections.json`、`assets/track.json`、`assets/curves/*.png`、`assets/track_vis.avi`。

### 校验

```bash
# Linux / macOS
shasum -a 256 2026-05-20_23-57-31.avi 2026-05-20_23-57-31.txt
# Windows PowerShell
Get-FileHash 2026-05-20_23-57-31.avi -Algorithm SHA256
```

结果应与上表一致。
