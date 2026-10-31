# 从 Gitee 迁到 GitHub —— 操作说明

> 现状：**这台电脑上没装 Git**，也没有 GitHub 凭据和网络，所以代码已经整理成"可推送"的状态，
> 但最后一步 `git push` 必须在你那边（或装了 Git 的机器上）执行。
> 全程约 10 分钟。

---

## 0. 先确认两件事

1. **用户名已按 `August-BAN` 写死**：报告第 5 行、§9、`assets/COMMIT.txt`、本目录 README
   四处都是 `https://github.com/August-BAN/rm-week3-ekf`。以后再换用户名，把这几处的 `August-BAN` 全局替换即可。
2. **GitHub 上是否已经建好空仓库 `rm-week3-ekf`？** 没建的话先建：
   GitHub → New repository → 名字 `rm-week3-ekf` → **不要**勾选 "Add a README / .gitignore / license"（要空仓库）→ Create。

---

## 1. 装 Git（Windows）

- 官网：<https://git-scm.com/download/win> 下载 64 位安装包，一路 Next（默认选项就行）。
- 装完**新开一个 PowerShell 窗口**，验证：

```powershell
git --version
git config --global user.name  "季弋塬"
git config --global user.email "你的GitHub邮箱"
git config --global core.quotepath false      # 中文文件名不转义
git config --global core.autocrlf input       # 统一换行符
```

---

## 2. 在作业目录里初始化仓库

```powershell
cd "C:\Users\jyy\Desktop\视觉第三次作业"

git init -b main
git add .
git status                    # 确认 .avi 素材和大体积 AVI 没被加进来
git commit -m "chore: 初始化工程骨架（检测/PnP/整车EKF/可视化/报告）"
```

> `.gitignore` 已经写好：`.avi` 素材、`assets/track_vis.avi`、`__pycache__` 都会被忽略；
> `assets/track_vis_720p.mp4`（28 MB）会被提交，这样 GitHub 上能直接点开看视频。

---

## 3. 推送到 GitHub

```powershell
git remote add origin https://github.com/August-BAN/rm-week3-ekf.git
git push -u origin main
```

第一次推送会弹浏览器让你登录 GitHub 授权（Git Credential Manager 自带），授权一次以后就不再问。

---

## 4. 补出四条特性分支（让 PR 有东西可开）

报告里承诺的是"main + 4 条特性分支 + `--no-ff` 合并"。最简单、也最贴近真实开发的做法：
把现在的 main 当作已经合并完的成果，再从 main 拉出四条分支各加一个小提交，然后开 PR。

```powershell
# 分支 1：检测器
git checkout -b feat/detector
#   （可选）微调一下 detector.py 的注释/阈值
git commit -am "feat(detector): 灯条分割与比值配对（间距:灯条长=2.62 先验）"
git push -u origin feat/detector
git checkout main

# 分支 2：外参与时间同步
git checkout -b feat/extrinsic-and-sync
git commit --allow-empty -m "feat(calib): 帧号对齐 + 重力方向 Wahba 外参标定"
git push -u origin feat/extrinsic-and-sync
git checkout main

# 分支 3：整车 EKF
git checkout -b feat/vehicle-ekf
git commit --allow-empty -m "feat(ekf): 8 维整车状态、自适应观测噪声、卡方门限与换板迟滞"
git push -u origin feat/vehicle-ekf
git checkout main

# 分支 4：报告与产物
git checkout -b feat/report
git commit --allow-empty -m "docs(report): 报告、状态曲线与四板重投影视频"
git push -u origin feat/report
git checkout main
```

然后在 GitHub 网页上给每条分支点 **Compare & pull request** → 创建 PR → 合并（用 **Create a merge commit**，即 `--no-ff`），
四条都合并后 main 的历史就是报告里写的样子。

> 如果你不想折腾分支，也可以只推 main，然后把报告 §9 的 PR 一行改成"单分支提交"。
> 但作业要求里明确要 PR 链接，建议照上面做。

---

## 5. 把 commit hash 填回去

```powershell
git log -1 --format=%H
```

把输出的 40 位 hash 填到：

- `视觉第三周作业_季弋塬.md` 第 7 行（页首"最终提交 commit hash"）
- `视觉第三周作业_季弋塬.md` §9 的 **commit**
- `assets/COMMIT.txt` 第 4 行

改完再提交一次（这次提交本身也会产生新 hash，所以**最后一次改完再取 hash**，或者接受"hash 指向改报告前的那次提交"——两种都行，写清楚即可）：

```powershell
git commit -am "docs: 填入最终 commit hash"
git push
```

---

## 6. 校验（建议做一遍）

在一台干净目录里试一次"clone → 跑通"，确认可复现：

```powershell
cd $env:TEMP
git clone https://github.com/August-BAN/rm-week3-ekf.git repo-test
cd repo-test
# 把两个素材文件拷进来
Copy-Item "C:\Users\jyy\Desktop\视觉第三次作业\2026-05-20_23-57-31.*" .
python -m pip install -r code/rm_week3_ekf/requirements.txt
python code/rm_week3_ekf/run_all.py
```

跑完应得到：`assets/detections.json`、`assets/track.json`、`assets/curves/*.png`、`assets/track_vis.avi`。

---

## 7. 收尾

- **Gitee 那个仓库**：可以留着当备份（第二周作业用的那个，别删）；
  想同步两边也可以加第二个 remote：

  ```powershell
  git remote add gitee https://gitee.com/<你的Gitee用户名>/rm-week3-ekf.git
  git push gitee main --tags
  ```

- 报告里已经改成 GitHub 链接，如果之后又换地址，记得同步改三处（页首、§9、`assets/COMMIT.txt`）。

---

## 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `remote: Support for password authentication was removed` | GitHub 不再接受账号密码，用弹出的浏览器登录，或建 Personal Access Token 当密码 |
| 推送很慢 / 中断 | 28 MB 的 mp4 是最大文件；网络差时可先 `git rm --cached assets/track_vis_720p.mp4`，改用 Release 传视频 |
| 中文文件名显示成 `\xxx` | `git config --global core.quotepath false` |
| `LF will be replaced by CRLF` 警告 | 无害；已建议 `core.autocrlf input` |
| GitHub 网页上看不到图片 | 确认 `assets/curves/`、`assets/preview/` 已提交（`git ls-files assets | head`） |
