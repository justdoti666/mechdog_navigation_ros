# HC-SR04 ×4 接 STM32（硬件捕获 → 串口上报）对接规格

> 状态：2026-08 定稿。路线 = 4 颗 HC-SR04 接底盘 STM32（定时器输入捕获），
> STM32 按新帧 `0x02` 经现有串口上报 → 师兄 `wheel_board_bridge_node` 解析后发布
> `/ultrasonic`（`mechdog_ultrasonic/UltrasonicArray`）→ `safety_node` **零改动**消费。
> 本文档给师兄：固件侧一页规格 + 桥节点补丁说明；接线图纸见 `ULTRASONIC_WIRING.md`
> （Pi 版供参考，STM32 版引脚由师兄按板子分配，电平转换一致）。

---

## 1. 为什么接 STM32

| 项 | 说明 |
|---|---|
| 树莓派限制 | Pi 5 Linux 环境**无可用硬件输入捕获**（SoC 有 PWM 捕获硬核但无驱动暴露） |
| STM32 优势 | 定时器输入捕获（亚 µs 精度、零 CPU 轮询、20Hz 无调度抖动） |
| 方案A 兼容 | 数据经 `/ultrasonic` 消息进算法库，**数据源可任意替换**，safety_node/算法库不改 |

---

## 2. 串口帧协议（STM32 → Pi，新增`0x02`）

与现有 `0x01`（设四轮 RPM）帧同构：`AA 55 | CMD | LEN | payload | checksum`，checksum = 帧头起至 payload 末所有字节和 `& 0xFF`。

```
AA 55 | 0x02 | 0x09 | FL_cm u16LE | FC_cm u16LE | FR_cm u16LE | BOT_cm u16LE | flags u8 | checksum u8
```
- 总长 **14 字节**；115200 波特率下 ≈1.3ms，10Hz 上报占用 <2% 带宽。
- 距离单位 **cm**（u16，小端），`0xFFFF` = 该路无效（超时/无回波）。
- `flags`（bit 0~3）：`FL/FC/FR/BOT_valid`，置 1 = 有效；与 `0xFFFF` 双保险。
- 字段顺序固定：**左前 / 正前 / 右前 / 底部**（与算法库 `get_ultrasonic_layout()` 一致）。

**STM32 发送建议**：测量 20Hz（50ms 周期），上报 10Hz（每 2 周期发一帧）；或直接 20Hz 上报亦兼容
（ROS 侧定时器按消息到达消费，不做限频）。

---

## 3. 固件侧规格（给师兄）

### 3.1 引脚分配（按师兄板子定，原则如下）
- **Echo ×4 → 定时器输入捕获**：推荐 **TIM2 CH1~CH4**（单定时器 4 通道，同一时钟基准）；
  或 4 个 TIM 各 1 通道。mode = 上升沿触发捕获 t1、下降沿捕获 t2，脉宽 = t2 - t1。
- **Trig ×4 → 普通 GPIO 推挽输出**：触发序列 = 低 ≥10µs → 高 10µs → 低。
- 4 路可**同时触发**（朝向不同无串扰），一次得到 4 个脉宽。
- 电平：Echo 5V → **1kΩ 串 + 2kΩ 对地分压（→3.3V）** → 捕获引脚；Trig 3.3V 直连；VCC 5V 共地。

### 3.2 测量时序
```
周期 50ms (20Hz):
  t0      全部 Trig 拉高 10µs 后拉低
  t0+ε    ARM 各通道捕获 (上升沿/下降沿中断)
  到 t1i  第 i 路上升沿捕获
  到 t2i  第 i 路下降沿捕获 → width_i = t2i - t1i
  25ms 硬超时: 无下降沿 → 该路无效
换算: cm = width_us / 58.0；有效区间 [2, 400]cm，越界/超时 → 0xFFFF + flag=0
```

### 3.3 注意
- 捕获中断里**只记计数器值**，换算/打包放主循环（或在中断里直接算亦可，量小）。
- 溢出：25ms @ 72MHz 主频 = 180 万 tick，16 位计数器 65536 会溢出 → 用 **uint32 累计**
  （TIM2/3/4/5 是 16 位，建议每通道记录时检查溢出计数；或直接用 32 位计数器 TIM2+TIM5 的 32 位模式）。
  简单方案：中断里 `width = (t2 - t1) & 0xFFFF` 会丢溢出帧，**必须处理**；或换算主频 1MHz 预分频（25ms 不溢出 16 位 65535µs？65535µs < 25ms 不够，用 100µs 计数: 25ms=250 计数, 无溢出问题，分辨率 0.1µs×58=… 换成 µs 级精度仍够：单计数 1µs @ 分频 72→ 预分频 72？ 1MHz tick: 25ms = 25000 < 65535 ✓ 分辨率 1µs（0.17mm）够用）。
  → **推荐预分频到 1µs/tick**，16 位计数器 25ms 内不溢出，实现最简单。
- 上报帧在底盘"发 RPM 帧"的同时**异步插入**即可（双向半双工无冲突；STM32 侧发送队列）。

---

## 4. ROS 侧整合（补丁，师兄的桥节点）

### 4.1 解析器（现成模块，零依赖纯 Python）
`scripts/ultrasonic_serial_parser.py` —— `UltrasonicFrameParser.feed(bytes) -> list[ParsedFrame]`，
自动处理分包/粘包/校验/超时无效值。本机自测：`python3 ultrasonic_serial_parser.py`。

### 4.2 桥节点改法（3 处小改动）
```python
# ① import
from mechdog_ultrasonic.msg import UltrasonicArray   # ament_python 运行时导入即可
from ultrasonic_serial_parser import UltrasonicFrameParser

# ② __init__: 建解析器 + /ultrasonic 发布器 (SensorDataQoS 与 safety_node 订阅一致)
self.ultra_parser = UltrasonicFrameParser()
self.ultra_pub = self.create_publisher(
    UltrasonicArray, '/ultrasonic', rclpy.qos.SensorDataQoS())

# ③ timer_callback 末尾: 每次调度都排空串口缓冲喂解析器 (独立于 read_reply 参数)
def drain_ultrasonic(self):
    if not self.serial_ok or self.serial is None:
        return
    try:
        n = self.serial.in_waiting
        if n > 0:
            data = self.serial.read(n)
            for f in self.ultra_parser.feed(data):
                msg = UltrasonicArray()
                msg.stamp = self.get_clock().now()
                msg.seq = f['seq']
                msg.period_sec = 0.1
                msg.front_left_cm = f['front_left_cm'];   msg.front_left_valid = f['front_left_valid']
                msg.front_center_cm = f['front_center_cm']; msg.front_center_valid = f['front_center_valid']
                msg.front_right_cm = f['front_right_cm'];  msg.front_right_valid = f['front_right_valid']
                msg.bottom_cm = f['bottom_cm'];            msg.bottom_valid = f['bottom_valid']
                self.ultra_pub.publish(msg)
    except Exception as exc:
        self.get_logger().warn(f'ultrasonic drain failed: {exc}')
```
### 4.3 注意
- **不要**同时启动 `ultrasonic_node`（模拟随机值）与 STM32 上报——两个发布者抢 `/ultrasonic`。
  真机只跑师兄桥节点（带补丁）；`ultrasonic_node` 仅用于 PC/WSL 模拟验证链路。
- `read_reply` 参数保持原语义（状态字符串显示），超声上报**不依赖**它。
- 时间戳用接收时刻（`self.get_clock().now()`），不启用 STM32 时钟同步。

---

## 5. 验收

1. 固件侧：`AA 55 02 ...` 帧 10Hz 稳定出现（串口助手可看）。
2. 桥节点：`ros2 topic echo /ultrasonic` 看到 4 路读数，手挡某颗该路变化。
3. `ros2 topic hz /ultrasonic` ≈ 10Hz。
4. 断一颗 Echo 连线 → 该路 `valid=false`，且 `is_fall_risk` 语义无变化（注入路径 fail-closed 已单测）。
