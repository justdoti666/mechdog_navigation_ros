# mechdog_navigation_ros — 局部安全层 ROS2 胶水包

将 `mechdog_navigation` 纯算法库（SensorFusion + PathPlanner）封装为 ROS2 节点，作为机械狗巡检的**局部安全层**，与师兄的全局定位/建图层（激光雷达 + AMCL + 栅格地图，ROS2 + 树莓派 5B）通过话题对接。

## 架构

```
师兄全局层 (Nav2)              本包 (局部安全层)
  /scan (雷达) ──────────────► safety_node
  /cmd_vel (Nav2 输出) ──────► (仲裁输入, 预留)
                                 │  SensorFusion.fuse()
                                 │  PathPlanner.plan()
                                 ▼
                              /cmd_vel (最终) ──► 底盘通信层 (2×STM32, 待定)
                              /fusion_result (调试/巡检决策)
```

## 目录结构

```
mechdog_navigation_ros/
├── package.xml            # ROS2 包描述 (Jazzy 默认; Humble 改 ROS_DISTRO 即可)
├── CMakeLists.txt         # ament_cmake, 子目录引用 ../mechdog_navigation 算法库
├── msg/FusionResult.msg   # 参考: 融合结果字段定义 (当前以 JSON 字符串发布, 未启用 rosidl)
├── src/safety_node.cpp    # 主节点: 融合+规划 -> /cmd_vel + /fusion_result
├── src/chassis_bridge.hpp # 底盘通信抽象: ChassisBridge 接口 + 模拟/STM32 实现
├── src/chassis_bridge_node.cpp  # 底盘桥接节点: 订阅 /cmd_vel -> 发送到底盘
├── launch/safety.launch.py
└── config/safety_params.yaml
```

## 依赖

- ROS2 (Jazzy / Humble), colcon
- `mechdog_navigation` 纯算法库源码（与本包**同级目录**，即 `../mechdog_navigation/`）
- 传感器真机驱动：Astra Pro（OpenNI2 待实现）、HC-SR04（libgpiod/pigpio，Pi 5 用）、环境光强（默认 `estimate_ambient_light()` 深度图代理，TSL2591 已取消购买）

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
| `/cmd_vel` | geometry_msgs/Twist | 发布 | safety_node 产出的最终速度指令 |
| `/cmd_vel` | geometry_msgs/Twist | 订阅 | chassis_bridge_node 消费, 发送到底盘 |
| `/fusion_result` | std_msgs/String (JSON) | 发布 | 融合结果 JSON（环境/悬崖/前方距离/动作/权重/速度） |
| `/scan` | sensor_msgs/LaserScan | 订阅 | 师兄雷达（当前缓存预留，不参与融合） |

## 对接注意

- **底盘通信**：`chassis_bridge_node` 订阅 `/cmd_vel`，通过 `ChassisBridge` 接口发送到底盘。默认 `bridge_type:=simulated`（日志打印，PC 可跑通链路）；真机改为 `bridge_type:=stm32`（`Stm32ChassisBridge` 待确认 2×STM32 通信方式后实现）。换实现只改 launch 参数，节点代码零改动。

- **ROS2 版本**：本包按 Jazzy（Ubuntu 24.04）写法；若师兄环境是 Humble（22.04），代码无需改，仅构建环境不同。
- **速度仲裁**：当前 safety_node 直接发布自己的规划结果。接入 Nav2 后，建议将 Nav2 的 `/cmd_vel` 作为输入，仅当本层检测到悬崖/近距障碍时覆盖输出（对应 `determine_action()` 悬崖最高优先级逻辑）。
- **底盘通信**：`/cmd_vel` → 2×STM32 的通信层尚未实现（第二个 STM32 用途待确认）。第一版可先 rqt 观察话题，或写一个简单的底盘模拟节点。
- **传感器真机化**：当前算法库内部走模拟数据；真机接入时需为 Astra/超声/光强各写 ROS2 驱动节点（或直接改算法库驱动层）。

## 状态

- [x] 包骨架 + safety_node 模拟模式
- [x] 底盘通信层骨架（ChassisBridge 接口 + 模拟实现，chassis_bridge_node）
- [ ] ROS2 版本确认（回学校后）
- [ ] 真机传感器节点（Astra OpenNI2 / HC-SR04 libgpiod）
- [ ] Stm32ChassisBridge 实现（待确认 2×STM32 通信方式）
- [ ] Nav2 速度仲裁接入
