# tools/ — 真机(树莓派)脚本与汇报工装

本目录收**在 Pi 上跑的运维/取证/汇报脚本**（节点源码在 `src/`，部署脚本见文末）。
所有脚本假定：`export ROS_DOMAIN_ID=42`，且已 `source /opt/ros/jazzy/setup.bash` + `~/mdw_ws/install/setup.bash`。

---

## 1 汇报窗口（可视化）

三格实时画面：**红外（与深度同视场） / 深度（JET 伪彩） / 可行度 2.5D**，
底部状态栏 `trav/up/down/steep/unknown + 平面 tilt/h0`。页面走 **MJPEG (:8080)**。

```bash
bash /home/chj/pi_view_up.sh        # 重启窗口服务（自动清旧进程与端口）
REC=30 bash /home/chj/pi_record.sh  # 录像 30s → /home/chj/report_YYYYmmdd_HHMMSS.mp4
```

浏览器：`http://<pi-ip>:8080/`（F11 全屏）。
**录像接口**：`RECORD_SECONDS=30`；标称帧率可用 `RECORD_FPS=25` 覆盖。

> **帧率已修正（v2.9.14, `af226b9`）**：旧版按固定 25fps 写、但渲染实际只有 ~19fps（降载后更低）
> ⇒ 每循环只写 1 帧 ⇒ 录 4s 只写 3s 的帧 ⇒ **播放快 1.34×**。
> 现改为**按真实时间间隔补齐帧数（累加器，余数留到下一轮）** ⇒ 录制时长 == 墙钟。
> 离线对照实验（真源码 exec，模拟渲染 19fps）：旧 75 帧/3.00s vs 墙钟 4.02s（−25.4%）；新 101 帧/4.04s vs 4.04s（−0.1%）。
> ⚠ 用 `round()` 而非累加器会失败（1.32 帧被舍成 1，误差永远修不回来）——这是第一版补丁的错，被对照实验抓出。

## 2 取帧 / 同帧验证（离线重算 + 节点逐字段对照）

```bash
python3 /home/chj/pi_frame_grab.py 5 平地     # 抓 5 帧 → /home/chj/frames_平地/
python3 /home/chj/pi_replay.py /home/chj/frames_平地 10 1    # 每帧 @10Hz 回放一遍
```

- **`pi_frame_grab.py`**：存 `depth_*.npy`（16UC1 原始深度，640×480）+ `caminfo.json`（K/D）+ `stamps.txt`（原 stamp / 本机时刻 / 有效像素占比）+ `node_line.txt`（同批节点日志的 `感知:` 行）。
- **`pi_replay.py`**：**沿用原时间戳**把帧灌回 `/camera/depth/image_raw` ⇒ 节点日志的 `感知:` 行可与离线计算**逐字段对齐**。
  - ⚠ 回放前**必须停掉真实相机**（`pkill -9 -f astra_camera_node`），否则两个深度源打架。
  - 兼容旧用法：`python3 pi_replay.py /home/chj/real_depth.raw 20`（单个 `.raw` 帧回放 20 秒）。

> 为什么必须"同一帧"：曾用**退化输入**（`plane=0` 的帧）去量成功路径耗时，结论全错 ⇒ 只有同一批像素才能当证据。

## 3 相机 / 无线 / 运维

| 脚本 | 用途 |
|---|---|
| `pi_cam_fix.sh` | 相机无数据时一键重启（kill + 重新拉起 astra_camera） |
| `pi_cam_ir.sh` | 关彩色 / 开红外（彩色流 UVC 故障时用） |
| `pi_stab.sh` | 查 WiFi 省电状态与话题速率（稳定性体检） |
| `pi_bringup_all.sh` | 一键拉起：相机 → 安全节点 → 窗口 |
| `pi_restore_verify.sh` | 恢复后核验（进程 / 话题 / 速率） |

## 4 取证 / 诊断（可复用的现场脚本）

| 脚本 | 用途 |
|---|---|
| `pi_gate_test.sh` | 深度质量守门行为验证（遮挡 / 半遮挡 / 条纹） |
| `pi_depth_health.py` | 30~60s 深度流健康采样：帧率 / 有效像素占比 / 全零帧 / 坏帧计数 |
| `pi_pitch_sweep.sh` | 俯仰角扫描（`cloud_pitch_rad` 0/15/25/35°）看 `in_fov` / `cov` / 假坑 |
| `pi_grid_check.sh` | `/safety/terrain_grid` 速率 / 带宽 / 延迟 + 与 `status_text` 直方图互证 |
| `pi_start_node.sh` | 起安全节点（含 `pkill` + 状态 / 感知 / 深度读数打印） |

## 5 标定

- `mount_calib.cpp`：离线三参数标定器（本地编译，用法见文件头注释）。
- 装机后流程见仓库外文档 `SOP_MOUNT_CALIB_ON_ROBOT.md`（含装夹复核判据 `tilt<2°` 且 `h0≈−镜头高`）。

## 6 部署

```bash
bash tools/deploy_all.sh        # 从本机把 tools/ 下的脚本推到 Pi 的 /home/chj/
```

---

## 已知问题与现状（2026-09-25）

- **彩色流无帧（UVC 通道故障）**：节点已注册 `/camera/color/image_raw`、日志 `color is started`，
  但 `topic hz` / 单帧 `echo` 取不到；降分辨率（320×240@15）与拔插 USB 均无效 ⇒ 用**红外**替代。
- **WiFi 掉线 / Pi 整机卡死**：根因已定案 = **`brcmfmac` SDIO 故障风暴**（一次开机日志刷 **1749 次**
  `brcmf_sdio_txfail: sdio error`，用户态随之全死：ICMP 偶通、但 22/8080 端口全超时）。
  诱因 `Power save: on`（重启后会回弹）⇒ 已用 systemd 服务 **持久关闭**（`wifi-powersave-off.service`）。
  该链路实测吞吐仅 **~1.25 Mbit/s**（`iw link` 显示 72 Mbit/s 是标称，实测只有 1.7%）⇒ 大流量遥测传不动，
  **遥测必须走紧凑话题**（`/safety/terrain_grid` 4.8 KB/帧 vs 旧彩图 233 KB/帧，小 48 倍）。根治：**插网线**。
- **深度偶发坏帧**：`valid=0.0% / 0 px` 的瞬时帧（30s 采样又 100% 健康，属间歇）。
  节点 v2.9.12 起 `status_text` 区分 **`DEPTH GATE (bad frame: …)`** 与 **`NO PLANE (fail-closed) …`**，
  现场一眼可辨是"数据坏"还是"构图坏"。

## 依赖

- 相机：`~/astra_ws`（astra_camera）；感知：`~/mdw_ws`（mechdog_navigation_ros）
- Python：numpy / cv2 / rclpy（Pi 上已具备）
- 环境：`ROS_DOMAIN_ID=42`
