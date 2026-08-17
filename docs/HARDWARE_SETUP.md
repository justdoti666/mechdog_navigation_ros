# mechdog_navigation 回学校操作清单（HARDWARE_SETUP）

> 生成日期：2026-08-08
> 适用：机械狗升级项目（Astra Pro 深度相机 + HC-SR04 + MLX90640 + 激光雷达 + 2×STM32 底盘）
> 状态：软件部分暑假已全部完成，此清单是回学校后的硬件接入与联调步骤

---

## 0. 前置：软件已就绪（无需重做）

| 项 | 状态 | 位置 |
|---|---|---|
| F2 真机深度驱动 | ✅ 完成 | mechdog_navigation（commit ed345af） |
| RGB 回传 + 深度叠加（DIST/NEAR） | ✅ 完成 | mechdog_navigation_ros（commit 7a25ddb） |
| ROS2 胶水包（safety_node + chassis_bridge_node） | ✅ 完成 | mechdog_navigation_ros |
| Linux ARM 版 Astra SDK | ✅ 已上传 | mechdog_navigation release v1.0.0-drivers（AstraSDK-v2.1.3-Linux-arm.zip 49.8MB） |
| 审查文档 | ✅ 完成 | mechdog_navigation/docs/ |

---

## 1. 树莓派 5B 环境准备

```bash
# 1.1 下载 Linux ARM 版 Astra SDK（已在 release）
wget https://github.com/justdoti666/mechdog_navigation/releases/download/v1.0.0-drivers/AstraSDK-v2.1.3-Linux-arm.zip
unzip AstraSDK-v2.1.3-Linux-arm.zip
# 记下解压路径，如 ~/astra_sdk

# 1.2 clone 两个仓库（必须同级！）
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
git clone https://github.com/justdoti666/mechdog_navigation.git
git clone https://github.com/justdoti666/mechdog_navigation_ros.git

# 1.3 确认 ROS2 环境（师兄的栈是 lyrical 还是其他？）
source /opt/ros/<distro>/setup.bash    # 按师兄实际版本

# 1.4 编译
cd ~/colcon_ws
colcon build --packages-select mechdog_navigation_ros
source install/setup.bash
```

---

## 2. Astra Pro 接入（深度 + RGB 回传）

**物理接线**：
- Astra Pro USB 线 → 树莓派 5B USB 口（一根线同时出深度 + RGB + 红外）

**代码对接**：
- `sensor_astra.cpp` 的 `capture_real()` 已用 Astra SDK 读深度（F2 完成）
- `rgb_stream.cpp` 已读 RGB + 深度叠加
- 编译时指定 SDK 路径：`cmake .. -DASTRA_SDK_ROOT=<解压路径>`

**验证**：
```bash
# 深度（F2 真机模式）
./mechdog_navigation   # main.cpp 里 astra(false)

# RGB 回传
./rgb_stream 8080
# 浏览器打开 http://<树莓派IP>:8080/stream，应看到画面 + DIST/NEAR 叠加
```

---

## 3. HC-SR04 ×4 接入（避障 + 防摔落）

**物理接线**（每个 HC-SR04 4 针，共 4 颗）：
```
HC-SR04 → 树莓派 GPIO
VCC     → Pin 2 (5V)
GND     → Pin 6 (GND)
TRIG    → GPIO17 / GPIO22 / GPIO23 / GPIO24   （左前/右前/正前/底部）
ECHO    → GPIO27 / GPIO25 / GPIO5  / GPIO6
⚠️ ECHO 是 5V 输出，必须分压到 3.3V 再接树莓派（电阻分压：2.2kΩ 串联 + 3.3kΩ 到地）
```

**代码对接**（关键：宏是编译开关，不是运行时发布）：
```bash
# sensor_ultrasonic.cpp 里 #ifdef USE_WIRINGPI 分支已写好 GPIO 逻辑
# 编译时打开宏：
cmake .. -DUSE_WIRINGPI=ON

# ⚠️ 树莓派 5B 注意：wiringPi 对 Pi5 (RP1芯片) 支持差
# 若编译/运行异常，改用 libgpiod：把 USE_WIRINGPI 换成 USE_GPIO（libgpiod），小改动
```

**验证**：真机模式下 `mechdog_navigation` 输出超声距离（不再是模拟值）。

---

## 4. 拆 deco 换 Astra Pro（保留高温检测）

**⚠️ 关键：deco 和 MLX90640 共享同一个黑色塑料支架，垂直堆叠**
- **只把 deco（CSI 摄像头）从支架上取下**
- **保留 MLX90640 和支架**——不要连支架一起拆，否则 MLX90640 会掉
- Astra Pro 装在原 deco 位置或旁边（USB 线接树莓派）
- MLX90640 原位保留 → 高温检测（打火机报警）不受影响

---

## 5. MLX90640 高温检测（保留，可选增强）

- **物理**：不动，原位保留（I2C 接口）
- **当前**：师兄系统已接入（红外全域测温 + 后台可视化），不用改
- **可选增强**：单独接树莓派 I2C（SDA/SCL），发布 `/thermal` 话题（32×24 温度矩阵），巡检检测设备过热

---

## 6. 激光雷达（师兄的，直接订阅）

- **物理**：不动
- **代码**：`safety_node.cpp` 已订阅 `/scan`（缓存预留），师兄 Nav2 的 `/scan` 话题直接可用

---

## 7. STM32 底盘通信（最后一块，需要师兄信息）

**前提**：确认第二个 STM32 的用途 + 树莓派↔STM32 通信方式（FDCAN / 串口）

**步骤**：
```bash
# 7.1 问师兄："底盘接收什么格式的速度指令？（线速度+角速度，还是别的协议？走 FDCAN 还是串口？）"
# 7.2 在 chassis_bridge.hpp 的 Stm32ChassisBridge::send_velocity() 里填发送代码
#     例如串口：serial.write("v=0.5,w=0.0\n")
# 7.3 launch 参数切真机
ros2 launch mechdog_navigation_ros safety.launch.py bridge_type:=stm32
```

**代码位置**：
- `mechdog_navigation_ros/src/chassis_bridge.hpp`（Stm32ChassisBridge 占位，TODO 注释处）

---

## 8. rgb_stream 树莓派平台适配（必做）

Windows 版用了 winsock + GDI+，树莓派（Linux）需替换：
- **winsock → Linux socket**（BSD socket，代码结构类似）
- **GDI+ 文字 → OpenCV putText 或 libfreetype**（深度叠加文字）
- 其余逻辑（Astra 读流、JPEG 编码）不变

---

## 9. 整机联调顺序（建议）

```
第1步：Astra Pro 深度 + RGB 真机跑通（验证 1+2）
第2步：HC-SR04 接上，真机距离正常（验证 3）
第3步：拆 deco 装 Astra，MLX90640 保留（验证 4）
第4步：手动遥控 + 遇障碍自动转弯（safety_node 仲裁，验证 1+3+7 底盘）
第5步：rgb_stream Linux 适配后，画面 + DIST/NEAR（验证 8）
第6步：自动巡逻（Nav2 规划 + safety 局部避障仲裁，需师兄 Nav2 栈配合）
```

---

## 待确认信息清单（找师兄/看实物）

- [ ] 树莓派 ROS2 具体版本（lyrical? humble?）
- [ ] 第二个 STM32 用途
- [ ] 树莓派 ↔ STM32 通信方式（FDCAN / 串口）及指令格式
- [ ] 师兄系统如何接收画面（协议/地址）→ 决定 rgb_stream 怎么对接
- [ ] HC-SR04 是否已有安装支架/位置

---

## 速查：两个仓库

| 仓库 | 内容 | 最新 commit |
|---|---|---|
| `justdoti666/mechdog_navigation` | 纯算法库（F2 真机驱动、融合、规划） | 以 `git log` 为准（示例历史: ed345af F2 → b84c8c7 文档 → 48a17e7 M 系列） |
| `justdoti666/mechdog_navigation_ros` | ROS2 胶水包（safety_node、chassis_bridge、rgb_stream） | 以 `git log` 为准（示例历史: 7a25ddb RGB+深度 → 3007099 ROS-5 补 → 3e62096 M 系列） |

**release v1.0.0-drivers**：AstraSDK-v2.1.3-Linux-arm.zip（树莓派）/ default.zip（Win 驱动）/ OpenNI_2.3.0.86_windows.zip（Win OpenNI2）
