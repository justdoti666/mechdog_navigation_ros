# mechdog_navigation_ros — 局部安全层 ROS2 胶水包

将 `mechdog_navigation` 纯算法库（SensorFusion + PathPlanner）封装为 ROS2 节点，作为机械狗巡检的**局部安全层**，与 `quadruped_ws` 全局栈（Nav2 + 激光雷达 + AMCL + 栅格地图 + 安全闸门 + STM32 底盘桥）通过话题对接，构成完整的三库系统。

## 三库架构总览

| 库 | 位置 | 职责 |
|---|---|---|
| **mechdog_navigation** | 本包同级 `../mechdog_navigation/` | 纯算法库（C++20，无 ROS2 依赖）：SensorFusion 多传感器融合（Astra 深度 + 4×HC-SR04 超声 + 红外环境）、PathPlanner 速度决策 |
| **mechdog_navigation_ros**（本包） | 任意 | 局部安全层 ROS2 节点：融合 → 规划 → 发布速度指令到安全闸门输入 |
| **quadruped_ws** | 全局层：Nav2 导航、cartographer/AMCL 建图定位、lidar 避障、语音/云台/热成像、双层速度闸门、STM32H723 串口底盘桥 |

### 速度指令控制链（安全分层）

```
Nav2 全局规划 ──► /unsafe/cmd_vel ──► [safety_gate_node 层1] ──► /cmd_vel
                                        │ estop / 超时0.5s / 限幅 ±0.08·±0.25
本包 safety_node ──► /unsafe/cmd_vel ──┘（与 Nav2 共用一个闸门输入, 安全统一）
                                                          │
                                          [wheel_board_bridge_node 层2]
                                          │ 超时 / estop / fault / /safety/state
                                          │ 限幅 ±0.20·±0.60 → 差速 → RPM
                                                          ▼
                                          STM32H723 (串口 21 字节帧) → 轮毂电机
```

**安全要点**：
- 本包**不直接发 `/cmd_vel`**（会绕过闸门），默认发布到 `/unsafe/cmd_vel`，由师兄 `cmd_vel_safety_gate_node` 统一安全检查后转发。可用 `-p cmd_vel_topic:=/cmd_vel` 覆盖（仅测试）。
- 侧双层闸门 + 状态新鲜度（`/safety/state`、`/robot/state` 1s 超时即零速，fail-closed）是最终安全兜底。
- 本包 5Hz 周期发送（>2Hz，满足师兄 0.5s 超时要求）。

### 底盘桥接两种接法（二选一）

1. **本包 Stm32ChassisBridge**（`bridge_type:=stm32`）：本包直接通过串口发 21 字节帧（`AA 55 0x01 0x10 + 4×float32 LE RPM + 和校验`，差速运动学轮径 0.0645m/轮距 0.256m），协议与 `wheel_board_bridge_node.py` 一致。
2. ** wheel_board_bridge_node 接管**（推荐，联调默认）：本包保持 `bridge_type:=simulated`，由师兄栈统一管串口，避免双写。

> 限幅分层说明：正常链路中安全闸门层1 先限幅（±0.08 m/s / ±0.25 rad/s），本桥限幅（±0.20/±0.60，与层2 同值）仅在直连模式（绕过闸门）下生效。排查限幅问题时先确认当前链路是哪一层在限。

## 目录结构

```
mechdog_navigation_ros/
├── package.xml            # ROS2 包描述 (Jazzy 默认; Humble 改 ROS_DISTRO 即可)
├── CMakeLists.txt         # ament_cmake, 子目录引用 ../mechdog_navigation 算法库
├── src/safety_node.cpp    # 主节点: 融合+规划 -> /unsafe/cmd_vel + /fusion_result
├── src/rgb_stream.cpp     # RGB 回传+深度叠加: Astra ColorStream -> MJPEG, 叠加 DIST(中央平均)/NEAR(最近障碍) (Windows-only, 独立 cl 编译, 未纳入 colcon 构建)
├── src/stb_image_write.h  # 单头文件 JPEG 编码库 (RGB 回传依赖)
├── src/chassis_bridge.hpp # 底盘通信抽象: ChassisBridge 接口 + 模拟/STM32(21字节帧) 实现
├── src/chassis_bridge_node.cpp  # 底盘桥接节点: 订阅 /cmd_vel -> 发送到底盘
└── launch/safety.launch.py
```

## 依赖

- ROS2 (Jazzy / Humble), colcon
- `mechdog_navigation` 纯算法库源码（与本包**同级目录**，即 `../mechdog_navigation/`）
- 传感器真机驱动：Astra Pro（Orbbec Astra SDK，真机经 `USE_ASTRA_SDK` 编译）、HC-SR04（libgpiod/pigpio，Pi 5 用）、环境光强（默认 `estimate_ambient_light()` 深度图代理，TSL2591 已取消购买）

## 构建 & 运行

```bash
# 工作区布局: colcon_ws/src/ 下同时放 mechdog_navigation 和 mechdog_navigation_ros
#   colcon_ws/src/mechdog_navigation/          # 纯算法库 (上游仓库)
#   colcon_ws/src/mechdog_navigation_ros/      # 本包
cd ~/colcon_ws
colcon build --packages-select mechdog_navigation_ros
source install/setup.bash

# 模拟模式 (无需硬件, PC 直接跑)
ros2 launch mechdog_navigation_ros safety.launch.py

# 真机模式
ros2 launch mechdog_navigation_ros safety.launch.py use_simulated:=false
```

## 话题接口

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/unsafe/cmd_vel` | geometry_msgs/Twist | 发布 | safety_node 产出速度指令 → 安全闸门输入（默认；`cmd_vel_topic` 可改） |
| `/cmd_vel` | geometry_msgs/Twist | 订阅 | chassis_bridge_node 消费, 发送到底盘（wheel_board_bridge 或本包 stm32 桥） |
| `/fusion_result` | std_msgs/String (JSON) | 发布 | 融合结果 JSON（环境/悬崖/前方距离/动作/权重/速度） |
| `/scan` | sensor_msgs/LaserScan | 订阅 | 雷达（sensor_data QoS, 当前缓存预留, 不参与融合） |
| `/mechdog/point_cloud` | sensor_msgs/PointCloud2 | 发布 | 近场深度点云（`camera_link` 系, `enable_pointcloud:=true` 启用, 默认关） |
| `/mechdog/negative_obstacles` | sensor_msgs/PointCloud2 | 发布 | **负障碍标记点**（`base_link` 系地面高度处, P1 地面分割检出坑/下行台阶; 跟随 `enable_pointcloud`） |
| `/mechdog/rgb/image_raw` | sensor_msgs/Image (rgb8) | 发布 | Astra 彩色帧（`enable_rgb:=true` 启用, 默认关；真机出图，默认 10fps，Foxglove bridge 自带压缩） |
| `/map` | nav_msgs/OccupancyGrid | 发布 | 占据栅格地图（仅 `mapping_demo_node`，1Hz；`odom_dry_run` 系） |

## 对接注意

- **安全闸门**：本包发布 `/unsafe/cmd_vel`，由 `cmd_vel_safety_gate_node` 统一安全检查（estop/超时/限幅）后转发 `/cmd_vel`。若想绕过闸门直发（仅测试）：`--ros-args -p cmd_vel_topic:=/cmd_vel`。
- **底盘通信**：`chassis_bridge_node` 订阅 `/cmd_vel`。默认 `bridge_type:=simulated`；真机 `bridge_type:=stm32` 时本包直接串口发 21 字节帧（协议同师兄）。**联调默认让 `wheel_board_bridge_node` 管串口，本包保持 simulated**，避免双写。
- **ROS2 版本**：本包按 Jazzy（Ubuntu 24.04）写法；若环境是 Humble（22.04），代码无需改，仅构建环境不同。
- **rgb_stream 安全**：`rgb_stream` 是无鉴权调试服务，绑定 `0.0.0.0`，局域网内任何设备都可查看摄像头画面与距离数据。仅限可信局域网调试使用，不要暴露到公网；如需长期运行建议改绑 `127.0.0.1`（改 `main` 中 `addr.sin_addr.s_addr`）或加反向代理鉴权。
- **速度仲裁**：当前 safety_node 直接发布自己的规划结果。接入 Nav2 后，建议将 Nav2 输出作为闸门输入的另一个发布者（师兄闸门天然支持多输入），本层仅在检测到悬崖/近距障碍时覆盖输出。
- **传感器真机化**：当前算法库内部走模拟数据；真机接入时需为 Astra/超声/光强各写 ROS2 驱动节点（或直接改算法库驱动层）。
- **前向全盲行为（接真机前必读）**：算法库在前向三方向全部失效（镜头被挡 + 三颗前向超声全坏）时输出 `SLOW_FORWARD` 降速盲行（仅 bottom 悬崖兜底），**不是 STOP**。接机械狗前务必与师兄闸门确认该场景有叠加保护；若本层是最后防线，按 mechdog_navigation README「已知限制」#7 把该分支改为 `STOP`。

## 近场点云（P3 起步）

**定位**：本包做**近场 3D 感知，补充激光雷达 2D 的盲区**（悬空障碍如桌沿、障碍物立体高度、悬崖边缘唇口）；全局建图定位归师兄栈（cartographer/AMCL + 激光雷达），本包不做地图、不碰 `map/odom`，全部在 `base_link` 局部系。

```bash
ros2 launch mechdog_navigation_ros safety.launch.py enable_pointcloud:=true
```

- 点云链路：深度图 → `depth_to_cloud` 反投影（0.6~8.0m 有效口径）→ `transform_optical_to_link` 固定旋转 → 每 N 点取 1 降采样 → 发布 `PointCloud2`（`enable_pointcloud` 默认 `false`，开启后行为不变只多一点云）
- 发布频率跟随融合节拍（真机 ~9-12Hz），话题 `/mechdog/point_cloud`，坐标系 `camera_link`（REP-103：X 前 Y 左 Z 上）
- launch 会同时启动 `static_transform_publisher`（`base_link → camera_link`，默认外参 x=0.12 / z=0.18 / pitch=+15°，**装机标定后用 `camera_x/camera_z/camera_pitch_rad` 覆盖**）
- 参数：`cloud_topic` / `cloud_frame` / `cloud_downsample_step`（默认 8，Pi 上算力紧可加大）

**师兄 Nav2 接入**（local_costmap 加一个 voxel 层即可消费）：

```yaml
voxel_layer:
  plugin: "nav2_costmap_2d::VoxelLayer"
  enabled: true
  origin_z: -0.2            # 相机装高 18cm, 地面以下(悬崖唇口)也要能标
  z_resolution: 0.05
  z_voxels: 16
  publish_voxel_map: true
  mark_threshold: 0
  observation_sources: pointcloud
  pointcloud:
    topic: /mechdog/point_cloud
    sensor_frame: camera_link
    data_type: "PointCloud2"
    expected_update_rate: 0.5   # 秒; 融合线程 ~9-12Hz, 0.5s 没更新即视为失效
    obstruction_max_range: 2.5  # 近场定位: 超过 2.5m 交给激光雷达
    raytrace_max_range: 3.0
    marking: true
    clearing: true
```

**注意**：

- **相机单进程独占**：safety_node 内部直接开 Astra（`use_simulated:=false` 时），不要再起第二个读相机的节点/程序，否则后开者 serial 为空、深度全失效（真机实测过）。点云跟着融合线程走正是为此。
- voxel_layer 标的是"有点的位置"：桌沿、立体障碍、**悬崖边缘唇口**都能标；整片"该有地面而没有"的负障碍（坑/下行楼梯）由 **P1 地面分割**输出到 `/mechdog/negative_obstacles`。

### 负障碍检测（P1）

- 原理：base_link 系受约束 RANSAC 拟合地面平面（高度先验带 + 法向竖直约束，防把墙当地面）→ 逐点分类 → 2.5D 栅格按列扫描，判据为**"地面 → 无回波带 → 更低一截"**；门口/Free space 后方地面同高则不标（单测 T4 试金石锁定）
- 话题：`/mechdog/negative_obstacles`（`base_link` 系，标记点位于地面平面高度）。建议师兄 costmap 给它单独一个 observation source 且 **`clearing: false`**——坑的标记不该被后续地面射线清掉
- 阈值：`cliff_drop_min = 0.12m`（mechdog_navigation `GroundSegConfig`；底部 HC-SR04 的 30cm 是紧急兜底阈值，两者语义不同勿混用）
- **fail-closed**：平面拟合失败（镜头没对着地面/外参不对）→ 不输出负障碍、真机模式 10s 节流告警；模拟模式数据是纯墙面，恒无负障碍属预期
- 依赖外参：`ground_prior_z = -0.18` 与 `CameraExtrinsics` 联动，装机量测后**两处同步改**

## RGB 回传整合（替代支架相机）

原支架上的可见光相机由 Astra Pro 的 RGB 一并承担（云台未落实，相机与 Astra 均为固定朝前，无视野损失），支架上仅保留红外热成像。整合动机：

- 减一颗相机：少一路 CSI/USB 占用、少一份供电与支架布局
- **RGB 与深度同源**：同一台 Astra 出的彩色与点云天然空间对齐，师兄的"温度-视觉"双重验证可升级为**温度-视觉-距离三重**（热成像报高温目标，点云直接给距离与高度）
- **全黑可用**：浓烟/黑暗下 RGB 失效，但结构光深度是主动投射，近场避障与点云仍工作

```bash
ros2 launch mechdog_navigation_ros safety.launch.py use_simulated:=false enable_rgb:=true enable_pointcloud:=true
```

- 话题 `/mechdog/rgb/image_raw`（sensor_msgs/Image, rgb8, 默认 10fps，`rgb_fps` 可调；Foxglove bridge 链路自带压缩，带宽无忧）
- 仅真机模式出图（模拟模式无彩色帧，参数开启不报错）
- 师兄侧改动：视觉源从支架相机话题切到本话题即可，"温度-视觉"管线本身不动


## 建图端到端演示（P4 demo）

`mapping_demo_node` — 验证「里程计位姿 → 点云 → 占据栅格」全链路的演示节点。深度为**合成帧**（模拟 3m 处横墙，内参按帧分辨率同比缩放），位姿为师兄 `base_cmd_vel_odom_node` 的真实输出，建图管线为算法库 P0/P4 真实代码。

```bash
# 终端1: 师兄 dry_run 里程计 (cmd_vel 开环积分, 位姿归零起步)
ros2 launch quadruped_base base_odom_dry_run.launch.py

# 终端2: 建图演示节点
ros2 run mechdog_navigation_ros mapping_demo_node

# 终端3: 驱动机器人 "行驶"
ros2 topic pub -r 10 /odom_test_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

# 查看: 地图话题 / PGM 落盘 ~/mechdog_map.pgm
ros2 topic echo /map --once | head -20
```

已验证（2026-09，WSL）：前进 0.6m + 原地转 1.2rad，PGM 占据点几何分布与解析模型**点级吻合**（墙带位置/转角扫掠轨迹/无背后异常点/首帧墙距含相机前偏）。

**定位边界**：本节点是建图链路的**演示/联调工具**，正式建图节点（Astra 真深度 + 真底盘 odom）待硬件接入后落地；生产全局建图定位仍归师兄栈（见上「近场点云」P3 定位声明，两者边界一致——demo 节点不改变近场感知定位）。

## 状态

- [x] 包骨架 + safety_node 模拟模式
- [x] 底盘通信层（ChassisBridge 接口 + 模拟实现 + Stm32ChassisBridge 21 字节帧实现）
- [x] 对接闸门（/unsafe/cmd_vel + sensor_data QoS）
- [x] 建图端到端演示（mapping_demo_node：dry_run odom + 合成深度 → /map + PGM，几何点级校验通过）
- [ ] ROS2 版本确认
- [ ] 真机传感器节点（Astra SDK / HC-SR04 libgpiod）
- [ ] Stm32ChassisBridge 串口实机联调（协议已实现, 待硬件验证）
- [ ] 建图真机化（Astra 真深度替换合成帧 + 真底盘 odom 位姿；Windows 侧静止/旋转扫描已由算法库 `tools/mapping_real_test` 验证）
- [ ] 真机部署前：确认前向全盲 `SLOW_FORWARD` 策略与全局闸门的兜底关系（mechdog_navigation README「已知限制」#7）
- [ ] Nav2 速度仲裁接入
