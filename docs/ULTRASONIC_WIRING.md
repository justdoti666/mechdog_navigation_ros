# 四路 HC-SR04 超声波接线方案（定稿）

> 适用：树莓派 40-pin 排针（Pi 4 / Pi 5 通用），4 颗 HC-SR04（左前 / 正前 / 右前 / 底部防跌）。
> 编号约定：**BCM GPIO 号 = libgpiod line offset**（代码里 `trig_pins`/`echo_pins` 全用 BCM 号，
> **不是** WiringPi 号、不是物理引脚号）。物理引脚号仅用于实际插线。
> 与代码对应：`mechdog_navigation/config.h::get_ultrasonic_layout()`、
> `mechdog_ultrasonic` 的 `ultrasonic_node` 参数 `trig_pins`/`echo_pins`。

---

## 1. 信号引脚分配（8 根信号线）

| 传感器 | 信号 | BCM GPIO | 物理引脚 | 代码索引 (trig_pins/echo_pins) |
|--------|------|----------|----------|-------------------------------|
| ① 左前 `front_left`   | Trig | GPIO 23 | **Pin 16** | `[0]` |
|                        | Echo | GPIO 24 | **Pin 18** | `[0]` |
| ② 正前 `front_center` | Trig | GPIO 17 | **Pin 11** | `[1]` |
|                        | Echo | GPIO 27 | **Pin 13** | `[1]` |
| ③ 右前 `front_right`  | Trig | GPIO 5  | **Pin 29** | `[2]` |
|                        | Echo | GPIO 6  | **Pin 31** | `[2]` |
| ④ 底部 `bottom`（朝下）| Trig | GPIO 13 | **Pin 33** | `[3]` |
|                        | Echo | GPIO 19 | **Pin 35** | `[3]` |

> 每对 Trig/Echo 物理相邻（16/18、11/13、29/31、33/35），杜邦线好走、好核对。

## 2. 电源与地

| 信号 | 引脚 | 说明 |
|------|------|------|
| VCC ×4 | **Pin 2、Pin 4**（5V） | HC-SR04 必须 5V 供电（3V3 会缩量程）；单颗峰值 ~30mA，4 颗 <150mA，Pi 5V 无压力 |
| GND ×4 | Pin 6 / 9 / 14 / 20 / 25 / 30 / 34 / 39 任选 | 每颗回地一根，**保证传感器与 Pi 共地**（不共地会误读） |

## 3. ⚠️ Echo 电平转换（5V → 3.3V，必做）

HC-SR04 的 Echo 输出是 **5V 逻辑**，Pi GPIO 耐压 3.3V（官方不推荐直接怼 5V；虽个别 Pi 可承受，工业巡检不能赌）。

**方案 A（推荐，零焊 8 个电阻）**：每路 Echo 一个分压器

```
Echo(5V) ──[1kΩ]──┬──> GPIO (3.3V)
                  └──[2kΩ]──┐
                           GND
```
- 分压比：`5V × 2k/(1k+2k) ≈ 3.3V`（正好在 3.3V 上限内，Pi 高电平阈值 2.31V，余量充足）
- 4 路共 8 个 1/4W 电阻；面包板或洞洞板一次焊好。
- （等效替代：2.2kΩ + 3.3kΩ → 3.0V，亦可用；下值一致即可）

**方案 B**：一块电平转换板（74AHCT125 / TXS0108E，4 路一次搞定，带 5V/3.3V 双电源脚）。

**Trig 不用转换**：Pi 3.3V 高电平 ≈ 3.3V > HC-SR04 触发电平门限（TTL VIH ≈ 2V），直接接。

## 4. 40-pin 排针使用表（图上核对用）

```
 Pin 1: 3V3        | Pin 2:  5V ⚡(VCC)
 Pin 3: GPIO2(I2C) | Pin 4:  5V ⚡(VCC)
 Pin 5: GPIO3(I2C) | Pin 6:  GND
 Pin 7: GPIO4      | Pin 8:  GPIO14(UART) ✗避
 Pin 9:  GND       | Pin 10: GPIO15(UART) ✗避
 Pin 11: GPIO17 ②Trig | Pin 12: GPIO18
 Pin 13: GPIO27 ②Echo | Pin 14: GND
 Pin 15: GPIO22    | Pin 16: GPIO23 ①Trig
 Pin 17: 3V3      | Pin 18: GPIO24 ①Echo
 Pin 19: GPIO10(SPI)✗ | Pin 20: GND
 Pin 21: GPIO9(SPI)✗  | Pin 22: GPIO25
 Pin 23: GPIO11(SPI)✗ | Pin 24: GPIO8(SPI)✗
 Pin 25: GND      | Pin 26: GPIO7(SPI CS1)✗
 Pin 27: GPIO0(ID)✗  | Pin 28: GPIO1(ID)✗
 Pin 29: GPIO5 ③Trig | Pin 30: GND
 Pin 31: GPIO6 ③Echo | Pin 32: GPIO12
 Pin 33: GPIO13 ④Trig| Pin 34: GND
 Pin 35: GPIO19 ④Echo| Pin 36: GPIO16
 Pin 37: GPIO26    | Pin 38: GPIO20
 Pin 39: GND       | Pin 40: GPIO21
```
（✗ = 避开的专用/保留脚：UART 8/10、I2C 3/5、SPI 19~24、ID 27/28——本机其他外设均走 USB，
GPIO 仅超声使用。）

## 5. 接线检查清单

1. **通断测一遍**：万用表蜂鸣档，按上表逐针核对（别按杜邦线颜色猜）。
2. **共地确认**：每颗 HC-SR04 的 GND 与 Pi GND 导通（不共地表现 = 距离恒 0 或恒满量程）。
3. **分压验证**：上电后量分压点对地 ≈ 3.3V（Echo 静态 0V 正常，触发时才出高）。
4. **软件验证**（Pi 上）：
   ```bash
   gpioinfo | grep -E '23|24|17|27|5|6|13|19'   # 确认引脚未被占用
   ros2 run mechdog_ultrasonic ultrasonic_node --ros-args -p use_gpio:=true
   ros2 topic echo /ultrasonic                    # 手挡传感器应见距离变化
   ```

## 6. 与代码的对应关系（改代码时只看这行）

- `config.h` layout 顺序 = 消息字段顺序：`front_left → front_center → front_right → bottom`
- `ultrasonic_node` 默认参数：`trig_pins=[23,17,5,13]`、`echo_pins=[24,27,6,19]`
  （顺序同上，与上表一一对应，**默认值无需改动**）
- 真机读取走 `USE_GPIO=ON` 编译（libgpiod）；`measure_hcsr04()` 实现尚未补全时节点发无效读数（fail-safe）。
