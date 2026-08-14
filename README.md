# mechdog_navigation_ros — 局部安全层 ROS2 胶水包

将 `mechdog_navigation` 纯算法库（SensorFusion + PathPlanner）封装为 ROS2 节点，作为机械狗巡检的**局部安全层**，与 `quadruped_ws` 全局栈（Nav2 + 激光雷达 + AMCL + 栅格地图 + 安全闸门 + STM32 底盘桥）通过话题对接，构成完整的三库系统。

## 三库架构总览

| 库 | 位置 | 职责 |
|---|---|---|
| **mechdog_navigation** | 本包同级 `../mechdog_navigation/` | 纯算法库（C++17，无 ROS2 依赖）：SensorFusion 多传感器融合（Astra 深度 + 4×HC-SR04 超声 + 红外环境）、PathPlanner 速度决策 |
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

## 目录结构

```
mechdog_navigation_ros/
├── package.xml            # ROS2 包描述 (Jazzy 默认; Humble 改 ROS_DISTRO 即可)
├── CMakeLists.txt         # ament_cmake, 子目录引用 ../mechdog_navigation 算法库
├── msg/FusionResult.msg   # 参考: 融合结果字段定义 (当前以 JSON 字符串发布, 未启用 rosidl)
├── src/safety_node.cpp    # 主节点: 融合+规划 -> /unsafe/cmd_vel + /fusion_result
├── src/rgb_stream.cpp     # RGB 回传+深度叠加: Astra ColorStream -> MJPEG, 叠加 DIST(中央平均)/NEAR(最近障碍)
├── src/stb_image_write.h  # 单头文件 JPEG 编码库 (RGB 回传依赖)
├── src/chassis_bridge.hpp # 底盘通信抽象: ChassisBridge 接口 + 模拟/STM32(21字节帧) 实现
├── src/chassis_bridge_node.cpp  # 底盘桥接节点: 订阅 /cmd_vel -> 发送到底盘
├── launch/safety.launch.py
└── config/safety_params.yaml
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

## 对接注意

- **安全闸门**：本包发布 `/unsafe/cmd_vel`，由 `cmd_vel_safety_gate_node` 统一安全检查（estop/超时/限幅）后转发 `/cmd_vel`。若想绕过闸门直发（仅测试）：`--ros-args -p cmd_vel_topic:=/cmd_vel`。
- **底盘通信**：`chassis_bridge_node` 订阅 `/cmd_vel`。默认 `bridge_type:=simulated`；真机 `bridge_type:=stm32` 时本包直接串口发 21 字节帧（协议同师兄）。**联调默认让 `wheel_board_bridge_node` 管串口，本包保持 simulated**，避免双写。
- **ROS2 版本**：本包按 Jazzy（Ubuntu 24.04）写法；若环境是 Humble（22.04），代码无需改，仅构建环境不同。
- **速度仲裁**：当前 safety_node 直接发布自己的规划结果。接入 Nav2 后，建议将 Nav2 输出作为闸门输入的另一个发布者（师兄闸门天然支持多输入），本层仅在检测到悬崖/近距障碍时覆盖输出。
- **传感器真机化**：当前算法库内部走模拟数据；真机接入时需为 Astra/超声/光强各写 ROS2 驱动节点（或直接改算法库驱动层）。

## 状态

- [x] 包骨架 + safety_node 模拟模式
- [x] 底盘通信层（ChassisBridge 接口 + 模拟实现 + Stm32ChassisBridge 21 字节帧实现）
- [x] 对接闸门（/unsafe/cmd_vel + sensor_data QoS）
- [ ] ROS2 版本确认
- [ ] 真机传感器节点（Astra SDK / HC-SR04 libgpiod）
- [ ] Stm32ChassisBridge 串口实机联调（协议已实现, 待硬件验证）
- [ ] Nav2 速度仲裁接入
