/**
 * ultrasonic_node — HC-SR04 超声波阵列 ROS2 节点 (方案A)
 *
 * 职责: 读 4 颗 HC-SR04 (左前/正前/右前/底部) 距离, 发布 /ultrasonic (UltrasonicArray)。
 *   供 mechdog_navigation_ros 的 safety_node 订阅 (不再由算法库内部直读 GPIO)。
 *
 * 读取:
 *   - 默认模拟模式 (WSL/PC 可跑通链路): 生成随机读数
 *   - USE_GPIO=ON (树莓派): libgpiod 读 Trig/Echo; 需 sudo 权限 + 电平转换 (5V→3.3V)
 *
 * 引脚 (ROS 参数; 值为 BCM GPIO 号 = libgpiod line offset, 非 WiringPi 号):
 *   默认对齐算法库 config.h get_ultrasonic_layout(), 物理引脚号见 docs/ULTRASONIC_WIRING.md:
 *   trig_pins  = [23, 17, 5, 13]  (BCM)  → 物理 Pin 16 / 11 / 29 / 33
 *   echo_pins  = [24, 27, 6, 19]  (BCM)  → 物理 Pin 18 / 13 / 31 / 35
 *   顺序 = [front_left, front_center, front_right, bottom]
 *   ⚠️ Echo 为 5V 输出, 必须 1kΩ+2kΩ 分压到 3.3V 再接 Pi (详见 ULTRASONIC_WIRING.md §3) *
 * 构建: colcon build --packages-select mechdog_ultrasonic
 * 运行: ros2 run mechdog_ultrasonic ultrasonic_node
 *       # 树莓派真读: colcon build ... -DUSE_GPIO=ON
 */
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "mechdog_ultrasonic/msg/ultrasonic_array.hpp"

#ifdef USE_GPIO
#ifdef LIBGPIOD_V2
#include <gpiod.h>   // libgpiod v2: gpiod_line_* API
#else
#include <gpiod.h>   // libgpiod v1: gpiod_* API
#endif
#include <unistd.h>
#endif

using namespace std::chrono_literals;
using UltraMsg = mechdog_ultrasonic::msg::UltrasonicArray;

namespace {

// 传感器方向名 (与消息字段 1:1)
const char* kNames[4] = {"front_left", "front_center", "front_right", "bottom"};

// 时钟: 高精度测 echo 宽度
double now_sec() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

#ifdef USE_GPIO
// ---- libgpiod 读一次 HC-SR04 (v1 与 v2 共用封装) ----
// 返回距离 cm; 超时/失败返回 NaN
double measure_hcsr04(int trig_chip, int trig_line, int echo_chip, int echo_line) {
    (void)trig_chip; (void)trig_line; (void)echo_chip; (void)echo_line;
    return std::numeric_limits<double>::quiet_NaN();  // 占位: 真实现按 libgpiod v1/v2 补全
}
#endif

} // namespace

class UltrasonicNode : public rclcpp::Node {
public:
    UltrasonicNode() : Node("ultrasonic_node") {
        // ---- 参数 ----
        use_gpio_ = this->declare_parameter("use_gpio", false);
        rate_hz_ = this->declare_parameter("rate_hz", 10.0);
        // ROS2 整数数组参数默认 int64 vector; 声明为 vector<int64_t> 避免类型不匹配
        trig_pins_ = this->declare_parameter("trig_pins", std::vector<int64_t>{23, 17, 5, 13});
        echo_pins_ = this->declare_parameter("echo_pins", std::vector<int64_t>{24, 27, 6, 19});

        // ---- 发布器 ----
        pub_ = this->create_publisher<UltraMsg>("/ultrasonic", rclcpp::SensorDataQoS());

        // ---- 定时器 (默认 10Hz) ----
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / std::max(rate_hz_, 1.0)),
            [this]() { publish_ultrasonic(); });

        RCLCPP_INFO(get_logger(), "ultrasonic_node 启动 (use_gpio=%d, rate=%.1fHz)",
                    use_gpio_ ? 1 : 0, rate_hz_);
    }

private:
    void publish_ultrasonic() {
        UltraMsg msg;
        msg.stamp = get_clock()->now();
        msg.seq = seq_++;
        msg.period_sec = static_cast<float>(1.0 / std::max(rate_hz_, 1.0));

       #ifdef USE_GPIO
        if (use_gpio_) {
            // 真机: libgpiod 读 (树莓派)
            // 注: 真实 HC-SR04 需要 Trig 拉低→脉冲→Echo 高电平宽度, 见 measure_hcsr04 注释
            for (int i = 0; i < 4; ++i) {
                (void)i;
                msg.front_left_cm = msg.front_center_cm = msg.front_right_cm = msg.bottom_cm =
                    std::numeric_limits<double>::quiet_NaN();
                msg.front_left_valid = msg.front_center_valid = msg.front_right_valid =
                    msg.bottom_valid = false;
            }
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "真实 GPIO 读取待补全 (见 measure_hcsr04), 当前发无效读数");
        } else
       #endif
        {
            // 模拟: 随机距离 (0.6~4m), 偶尔无效 (fail-closed 测试)
            simulate(&msg);
        }

        pub_->publish(msg);
    }

    void simulate(UltraMsg* msg) {
        // 简化: 各方向 0.5~3.5m 随机; 90% 有效
        auto rnd = [this](double lo, double hi) {
            std::uniform_real_distribution<double> d(lo, hi);
            return d(gen_);
        };
        // 4 颗: 左前/正前/右前/底部
        msg->front_left_cm = rnd(50.0, 350.0);
        msg->front_center_cm = rnd(50.0, 350.0);
        msg->front_right_cm = rnd(50.0, 350.0);
        msg->bottom_cm = rnd(10.0, 30.0);   // 底部贴近地面
        msg->front_left_valid = rnd(0.0, 1.0) > 0.05;
        msg->front_center_valid = rnd(0.0, 1.0) > 0.05;
        msg->front_right_valid = rnd(0.0, 1.0) > 0.05;
        msg->bottom_valid = rnd(0.0, 1.0) > 0.05;
        (void)rnd;
    }

    rclcpp::Publisher<UltraMsg>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint32_t seq_ = 0;
    bool use_gpio_ = false;
    double rate_hz_ = 10.0;
    std::vector<int64_t> trig_pins_, echo_pins_;
#ifdef USE_GPIO
    // libgpiod state...
#endif
    std::mt19937 gen_{static_cast<unsigned>(std::random_device{}())};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UltrasonicNode>());
    rclcpp::shutdown();
    return 0;
}
