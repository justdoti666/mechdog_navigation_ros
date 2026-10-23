# tools/ — 真机(树莓派)脚本与汇报工装

本目录收**在 Pi 上跑的运维/取证/汇报脚本**（节点源码在 `src/`，部署脚本见文末）。
所有脚本假定：`export ROS_DOMAIN_ID=42`，且已 `source /opt/ros/jazzy/setup.bash` + `~/mdw_ws/install/setup.bash`。

---

## 1 汇报窗口（可视化）

三格实时画面：**红外（与深度同视场） / 深度（JET 伪彩） / 可行度 2.5D**，
底部状态栏 `trav/up/down/steep/unknown + 平面 tilt/h0`。页面走 **MJPEG (:8080)**。

> **深度面板数据源已换（v2.9.15, `3ae3634`）**：窗口不再订阅 **614KB 原始深度**，改订阅节点发的
> **`/safety/depth_small`**（16UC1 320×240 = 1/2 抽点，≈154KB/帧）。节点侧参数 `publish_depth_small`（默认 true）；
> 关掉它、或窗口/节点版本不配对时，深度面板显示 `no depth yet (need /safety/depth_small, node v2.9.15+)`。
> **部署时两处必须同步**：`bash tools/deploy_all.sh` 推脚本 + 重启节点。
> 等价性（本地 WSL 验证）：小图与源帧抽点**逐像素一致**；新旧窗口渲染**整帧 720×612 = 0/440,640 像素差异**（JPEG 逐字节相同）。

```bash
bash ~/pi_view_up.sh                # 重启窗口服务（自动清旧进程与端口）
REC=30 bash ~/pi_record.sh          # 录像 30s → ~/report_YYYYmmdd_HHMMSS.mp4
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
python3 ~/pi_frame_grab.py 5 平地     # 抓 5 帧 → ~/frames_平地/
python3 ~/pi_replay.py ~/frames_平地 10 1    # 每帧 @10Hz 回放一遍
```

- **`pi_frame_grab.py`**：存 `depth_*.npy`（16UC1 原始深度，640×480）+ `caminfo.json`（K/D）+ `stamps.txt`（原 stamp / 本机时刻 / 有效像素占比）+ `node_line.txt`（同批节点日志的 `感知:` 行）。
- **`pi_replay.py`**：**沿用原时间戳**把帧灌回 `/camera/depth/image_raw` ⇒ 节点日志的 `感知:` 行可与离线计算**逐字段对齐**。
  - ⚠ 回放前**必须停掉真实相机**（`pkill -9 -f astra_camera_node`），否则两个深度源打架。
  - 兼容旧用法：`python3 pi_replay.py ~/real_depth.raw 20`（单个 `.raw` 帧回放 20 秒）。
- **帧指纹对照（R-8，2026-10-15）**：节点「感知:」行尾现带 `| 帧fp=<8hex>`（空/模拟帧 `--------`），与离线工装同实现同口径 ⇒ 用 core 仓 `tools/wedge_compare.py` **按指纹配对**做逐字符对照（`final_*.txt` + 日志；3000ms 节流的重复行自动去重）。
  - ⚠ 回放必须发布 `/camera/depth/camera_info`（`pi_replay.py` 已带）。**漏发 ⇒ 节点落 FOV 兜底内参（572.7/320.0）**，与工装硬编码（570.342…/319.5）不同 ⇒ 平面拟合**假分歧**（同帧实测 tilt 8.93° vs 2.46°，2026-10-15 定位）。

> 为什么必须"同一帧"：曾用**退化输入**（`plane=0` 的帧）去量成功路径耗时，结论全错 ⇒ 只有同一批像素才能当证据。

### 2.1 A 批「材质指纹对照实验」工具（2026-10-15）

```bash
export ROS_DOMAIN_ID=42                      # ★ Pi 上节点在这个 domain，不设会"看不到话题"
python3 ~/hidden_probe.py --preflight --terrain                     # 前置：链路/超声/深度在不在
python3 ~/hidden_probe.py --label ~/label.json --out ~/a_mat.jsonl \
        --terrain --secs 10 --hz 2                                  # 每个测点采 10s（label 每轮重读）
python3 ~/hidden_summarize.py --in ~/a_mat.jsonl --out ~/矩阵.md      # 出「材质×传感器」指纹矩阵
```

- **`hidden_probe.py`**：把**三层对同一块材质的反应**按帧配对存档 —— 超声原始读数（`/ultrasonic`）+
  节点判据真值（`/fusion_result` 的 `hidden_obstacle*`）+ 深度层独立证据（`/safety/terrain_grid` 复算
  "该方向距离带内禁行格数"，几何与 `HeightMap25Config` 同口径：`wx=min_x+(c+.5)cell`、`wy=-y_half+(r+.5)cell`、
  禁行格 = 1/2/4、Unknown 不算）。
  **本脚本不重新实现判据**（窗 [0.90,3.00]m / 带 ±0.25m / 扇区 ±25° 由节点自己算），只做配对与"深度侧是否有挡格"的旁证。
- **`hidden_summarize.py`**：按 `label.json` 分组出矩阵（`○`/`·`/`?`）+ 逐测点明细 + 判读建议草稿。
- **协议**（现场照着做）：[`docs/材质对照实验协议_2026-10-15.md`](../docs/材质对照实验协议_2026-10-15.md)
  —— 含 5 材质×3 距离矩阵、每测点 5 步、验收判据、**已知坑表**（domain 42 / camera_info / 板贴地 / 距离出窗 / 地形未注入 …）。

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
| `pi_gate_test.sh` | 深度质量守门**真机注入回归 v2**（2026-09-29, T6）：造全 0 帧 → 真注入(`pi_replay` 10Hz) → 断言（启动期降级/守门行 valid=0.0%/降级路径）→ PASS/FAIL+退出码；**不依赖相机硬件** |
| `ultra_gate_check.sh` | **B4 超声启动门回归**（无需硬件）：`silent`/`once` 两种发布者场景，验证 等待首帧 → 重新接入 → 退出安全链 日志序列 |
| `ultra_failclosed_check.sh` | **U-1 fail-closed 回归**（2026-10-05）：无可用 GPIO 环境起节点（`use_gpio:=true`）→ 断言 `/ultrasonic` 帧内**零** `_valid: true`（拿不到 GPIO 时绝不发模拟随机数）；真机 GPIO 可用时自动 SKIP |
| `ultra_capture.sh` | **U-6 真机取证**（2026-10-05）：抓 N 帧 `/ultrasonic` + 节点进程/日志/环境 META → 输出目录台账（`MANIFEST.md5` 可核；新鲜度只认 md5 不认 mtime；前提：节点已在跑） |
| `ultra_frame_capture.py` | 上面两个工装的**帧采集内核**（rclpy 直捕单帧，绕开 Ubuntu-26.04/lyrical `ros2 topic echo` 的 XMLRPC 兼容问题；jazzy/lyrical 双发行实测可用） |
| `pi_start_ultrasonic.sh` | **真机超声启动（R-2，2026-10-11）**：显式带 `use_gpio:=true`（+ 可选 `ACTIVE=1` 单颗台架）起 `ultrasonic_node`，避免"忘传参数 ⇒ 节点发模拟随机数且 `valid=true` 而下游认不出"；起完自动打印进程/日志关键行/一帧读数。<br>⚠ 只在 **Pi GPIO 路线**用；主路线（STM32 桥发布 `/ultrasonic`）**不要**同时起本节点 |
| `pi_depth_health.py` | 30~60s 深度流健康采样：帧率 / 有效像素占比 / 全零帧 / 坏帧计数 |
| `pi_window_ab.sh` | **#9 真机验收**：窗口（旧/新）× 节点（`publish_depth_small` 关/开）四相位 CPU 对比 |
| `pi_pitch_sweep.sh` | 俯仰角扫描（`cloud_pitch_rad` 0/15/25/35°）看 `in_fov` / `cov` / 假坑 |
| `pi_grid_check.sh` | `/safety/terrain_grid` 速率 / 带宽 / 延迟 + 与 `status_text` 直方图互证 |
| `pi_start_node.sh` | 起安全节点（含 `pkill` + 状态 / 感知 / 深度读数打印） |

## 5 标定

- `mount_calib.cpp`：离线三参数标定器（本地编译，用法见文件头注释）。
- v2.9.18 加固：yaw 不可辨识时**不给建议值**（单水平面观测不出）；坏帧自动剔除；缺值/非法参数与 `--step 0` 直接报错（不再静默吞参/死循环）。
- **双平面 + 自检（2026-10-04, 离线待办 #1）**：`--wall` 目标函数追加**墙项**（对地面以上点受约束 RANSAC 拟竖直面，墙拟合失败 ⇒ 候选作废；墙项不增加可辨识性，是第二结构面复核）；`--selftest` 自造"地板+墙"深度帧走同一搜索链做真值断言，**12 工况 74/74 通过**（验收 姿态 <0.5°、h0 <5cm）。
- **抗密度偏置抽稀（仅标定器内部，节点侧不动）**：按 (x,y) 5cm 格均匀抽稀再进拟合（每格取首末端点中点；多表面堆叠格如墙压地板取较低者）——消除"近密远疏 + 逐格最低面"带来的 +0.25~1.0° 俯仰系统偏置，及单端取样的格边界噪声条件化偏差（±0.29°）。
- 装机后流程见仓库外文档 `SOP_MOUNT_CALIB_ON_ROBOT.md`（含装夹复核判据 `tilt<2°` 且 `h0≈−镜头高`）。

## 6 部署

```bash
bash tools/deploy_all.sh        # 完整部署: 核心改动 + ROS 包 + tools/*.sh|*.py → Pi 的 ~/
```

---

## 7 L0 速度仲裁 A/B（离线，无需硬件）

```bash
WS=~/mdw_ws bash ~/mdw_ws/src/mechdog_navigation_ros/tools/arbiter_ab.sh
```

- 判据（`arbiter_enable:=true`）：① 每帧 `|vx| ≤ min(期望, 0.20)`（只减不增）② 出现过 0.15(A) / 0.20(B)（期望确实被采用）③ 期望停发 >300ms ⇒ 零速。
- `arbiter_enable:=false` 只记录对照：期望速度应**完全无影响**（输出仍由本节点 planner 决定）。
- 两个模式各起停一次 `safety_node`，**按进程名清理**（`ros2 run` 只是启动器，`kill $!` 会留下子进程继续发 `/l0/out`，混流会污染结论 —— 首版踩过）。
- 输出：`$OUT/probe_{true,false}.txt` + 样本 CSV + 节点日志（默认 `$HOME/arbiter_ab_<日期>`）。
- WSL 实测（2026-10-23，Jazzy）：true = PASS（A `0.150` / B `0.200` / C `0.000`）；false = PASS（期望无影响）。真机上把 `use_simulated` 改 `false` 即可复跑同一判据。

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

## 窗口 CPU 归账（2026-09-25, 本地 x86 微基准，供优化决策)

| 分量 | 成本 | 说明 |
|---|---|---|
| 深度回调（旧） | 1.342 ms/帧 | 掩码+JET+缩放 **1.17 ms 占大头**；614KB→float32 只有 0.091 ms |
| 深度回调（新） | 1.245 ms/帧 | 去掉全分辨率转换 ✓（`/safety/depth_small` 方案） |
| IR 回调 | **0.866 → 0.801 ms/帧** | **已做**（`v2.9.16`）：「先抽点再转 float32」**−7%**、输出逐像素相同。（早前写的 "−52%" 是错的基准 ⇒ 已更正，见下注） |
| compose+JPEG | 1.493 ms/帧 | 两版相同；受渲染帧率影响最大 |

⇒ 深度通道换小图的收益**主要在带宽**（−75%/帧），不在 Python CPU；想再降窗口 CPU，杠杆在 **IR 路径**。

> **基准更正（2026-09-25）**：早前 "IR 先抽点变体 −52%" 的结论**作废** —— 那份基准里的"现状"是我凭记忆复现的，顺序写反（真代码本来就是先抽点后裁剪）⇒ 用**真文件对照**（`git show HEAD:tools/pi_report_view.py` 导入旧模块）后为 **−7%**。教训：**性能对照的基线必须是真源码，不能是自己复现的版本。**

## 依赖

- 相机：`~/astra_ws`（astra_camera）；感知：`~/mdw_ws`（mechdog_navigation_ros）
- Python：numpy / cv2 / rclpy（Pi 上已具备）
- 环境：`ROS_DOMAIN_ID=42`
