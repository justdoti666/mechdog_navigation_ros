/**
 * ultrasonic_node — HC-SR04 超声波阵列 ROS2 节点 (方案A)
 *
 * 职责: 读 4 颗 HC-SR04 (左前/正前/右前/底部) 距离, 发布 /ultrasonic (UltrasonicArray)。
 *   供 mechdog_navigation_ros 的 safety_node 订阅 (不再由算法库内部直读 GPIO)。
 *
 * 读取:
 *   - 默认模拟模式 (WSL/PC 可跑通链路): 生成随机读数
 *   - USE_GPIO=ON (树莓派): libgpiod 真读 Trig/Echo
 *     · Ubuntu 24.04 (Pi 5B) 自带 libgpiod v1.6.3 ⇒ 默认 v1 路径; v2 系统加 -DLIBGPIOD_VERSION=v2
 *     · 芯片自动探测 (gpio_chip:="auto"): label 含 "rp1"(Pi 5) > "bcm"(Pi 4) > /dev/gpiochip4 > /dev/gpiochip0
 *     · 引脚值为 BCM 号 = libgpiod line offset; 5V 供电时 Echo 必须分压到 3.3V (docs/ULTRASONIC_WIRING.md §3)
 *     · chj 在 dialout 组, /dev/gpiochip4 直开, 无需 sudo
 *
 * 引脚 (ROS 参数; 值为 BCM GPIO 号 = libgpiod line offset, 非 WiringPi 号):
 *   trig_pins = [23, 17, 5, 13] (BCM) → 物理 Pin 16 / 11 / 29 / 33
 *   echo_pins = [24, 27, 6, 19] (BCM) → 物理 Pin 18 / 13 / 31 / 35
 *   顺序 = [front_left, front_center, front_right, bottom]
 *   ⚠️ Echo 为 5V 输出, 5V 供电时必须 1kΩ+2kΩ 分压到 3.3V 再接 Pi (详见 ULTRASONIC_WIRING.md §3)
 *
 * 只接部分传感器 (例: 先接正前一颗, 其余悬空):
 *   ros2 run mechdog_ultrasonic ultrasonic_node --ros-args -p use_gpio:=true -p 'active:=[1]'
 *   active = 参与轮询的通道下标子集 (默认 [0,1,2,3]); 未启用通道发布 invalid (NaN)
 *
 * 构建: colcon build --packages-select mechdog_ultrasonic --cmake-args -DUSE_GPIO=ON
 * 运行: ros2 run mechdog_ultrasonic ultrasonic_node --ros-args -p use_gpio:=true
 */
#include <algorithm>
#include <array>
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
#include <gpiod.h>
#include <unistd.h>
#endif

using namespace std::chrono_literals;
using UltraMsg = mechdog_ultrasonic::msg::UltrasonicArray;

namespace {

// 与核心库 config.h UltrasonicConfig 同口径 (两处漂移会对不上离线基准):
constexpr double kSpeedOfSoundCmPerSec = 34300.0;  // 声速 cm/s @20°C
constexpr double kMinValidCm           = 2.0;      // 物理量程下界
constexpr double kMaxValidCm           = 400.0;    // 物理量程上界 (4m)
constexpr int    kEchoTimeoutMs        = 25;       // 单边沿等待上限 (4m 往返 ~23.3ms; 对齐核心库 timeout 25ms)
constexpr int    kMaxChannels          = 4;        // FL / FC / FR / BOT
constexpr int    kTrigPulseUs          = 15;       // 触发脉宽 (HC-SR04 要求 >=10µs)

#ifdef USE_GPIO
// ---- 自动找 40-pin 排针所在 gpiochip (扫 label) ----
// Pi 5 (RP1): label 含 "rp1" ⇒ /dev/gpiochip4; Pi 4 (bcm2711): 含 "bcm"。
// 都找不到 → 返回空串, 由调用方走保底路径。
std::string detect_header_chip_path() {
    std::string bcm_fallback;
    for (int n = 0; n <= 9; ++n) {
        std::string path = "/dev/gpiochip" + std::to_string(n);
        struct gpiod_chip* c = gpiod_chip_open(path.c_str());
        if (!c) continue;
#ifdef LIBGPIOD_V2
        std::string lab;
        struct gpiod_chip_info* info = gpiod_chip_get_info(c);
        if (info) {
            const char* label = gpiod_chip_info_get_label(info);
            lab = label ? label : "";
            gpiod_chip_info_free(info);
        }
#else
        const char* label = gpiod_chip_label(c);
        std::string lab = label ? label : "";
#endif
        gpiod_chip_close(c);
        if (lab.find("rp1") != std::string::npos) return path;
        if (bcm_fallback.empty() && lab.find("bcm") != std::string::npos) bcm_fallback = path;
    }
    return bcm_fallback;  // 可能为空
}
#endif

} // namespace

#ifdef USE_GPIO
// ---- HC-SR04 读取器: 一次性占好各通道 Trig(输出低)/Echo(双边沿输入+下拉) ----
// 说明: 逐路串行测距; 未接线的通道 echo 无回波 ⇒ 超时 ⇒ 该路 NaN (fail-closed)。
//       下拉偏置保证悬空 Echo 不会飘出假边沿。
class Hcsr04Reader {
public:
    Hcsr04Reader(const std::vector<int64_t>& trig_pins,
                 const std::vector<int64_t>& echo_pins,
                 const std::string& chip_sel)
    {
        const size_t n = std::min({trig_pins.size(), echo_pins.size(), (size_t)kMaxChannels});
        if (n == 0) return;

        std::string path;
        if (!chip_sel.empty() && chip_sel != "auto") {
            path = chip_sel;
        } else {
            path = detect_header_chip_path();
            if (path.empty()) path = "/dev/gpiochip4";
        }
        chip_ = gpiod_chip_open(path.c_str());
        if (!chip_ && path != "/dev/gpiochip0") {   // 保底: 老机型排针在 gpiochip0
            path = "/dev/gpiochip0";
            chip_ = gpiod_chip_open(path.c_str());
        }
        if (!chip_) return;
        chip_path_ = path;
#ifdef LIBGPIOD_V2
        {
            struct gpiod_chip_info* info = gpiod_chip_get_info(chip_);
            if (info) {
                const char* label = gpiod_chip_info_get_label(info);
                chip_label_ = label ? label : "";
                gpiod_chip_info_free(info);
            }
        }
#else
        {
            const char* label = gpiod_chip_label(chip_);
            chip_label_ = label ? label : "";
        }
#endif

        for (size_t i = 0; i < n; ++i) {
            trig_off_[i] = (unsigned)trig_pins[i];
            echo_off_[i] = (unsigned)echo_pins[i];
            bool ok = false;
#ifdef LIBGPIOD_V2
            ok = request_v2_(i);
#else
            trig_line_[i] = gpiod_chip_get_line(chip_, trig_off_[i]);
            echo_line_[i] = gpiod_chip_get_line(chip_, echo_off_[i]);
            if (trig_line_[i] && echo_line_[i]) {
                ok = (gpiod_line_request_output(trig_line_[i], "ultrasonic_node", 0) == 0);
                if (ok) {
                    ok = (gpiod_line_request_both_edges_events_flags(
                              echo_line_[i], "ultrasonic_node",
                              GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN) == 0);
                }
            }
#endif
            line_ok_[i] = ok;
        }
    }

    ~Hcsr04Reader() {
#ifdef LIBGPIOD_V2
        if (echo_req_) gpiod_line_request_release(echo_req_);
        if (trig_req_) gpiod_line_request_release(trig_req_);
        if (ev_buf_)   gpiod_edge_event_buffer_free(ev_buf_);
#else
        for (auto* l : echo_line_) if (l) gpiod_line_release(l);
        for (auto* l : trig_line_) if (l) gpiod_line_release(l);
#endif
        if (chip_) gpiod_chip_close(chip_);
    }

    bool chip_open() const { return chip_ != nullptr; }
    const std::string& chip_path()  const { return chip_path_; }
    const std::string& chip_label() const { return chip_label_; }

    // 返回距离 cm; 超时/失败/超量程 返回 NaN
    double measure(size_t i) {
        if (!chip_ || i >= (size_t)kMaxChannels || !line_ok_[i]) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        // 触发: 低(维持) → 高 kTrigPulseUs → 低
        set_trig_(i, true);
        usleep(kTrigPulseUs);
        set_trig_(i, false);

        uint64_t t_rise = 0, t_fall = 0;
        if (!wait_edge_(i, /*rising=*/true, t_rise))  return std::numeric_limits<double>::quiet_NaN();
        if (!wait_edge_(i, /*rising=*/false, t_fall)) return std::numeric_limits<double>::quiet_NaN();
        if (t_fall <= t_rise) return std::numeric_limits<double>::quiet_NaN();

        double cm = (double)(t_fall - t_rise) * (kSpeedOfSoundCmPerSec / 2.0) / 1e9;
        cm = std::round(cm * 100.0) / 100.0;
        if (cm < kMinValidCm || cm > kMaxValidCm) return std::numeric_limits<double>::quiet_NaN();
        return cm;
    }

private:
#ifdef LIBGPIOD_V2
    bool request_v2_(size_t i) {
        // Trig: 输出, 初始低
        struct gpiod_line_settings* s = gpiod_line_settings_new();
        if (!s) return false;
        gpiod_line_settings_set_direction(s, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(s, GPIOD_LINE_VALUE_INACTIVE);
        size_t off = trig_off_[i];
        struct gpiod_line_config* lc = gpiod_line_config_new();
        if (!lc) { gpiod_line_settings_free(s); return false; }
        bool ok = (gpiod_line_config_add_line_settings(lc, &off, 1, s) == 0);
        struct gpiod_request_config* rc = gpiod_request_config_new();
        if (!rc) { gpiod_line_config_free(lc); gpiod_line_settings_free(s); return false; }
        gpiod_request_config_set_consumer(rc, "ultrasonic_node");
        if (ok) {
            trig_req_ = gpiod_chip_request_lines(chip_, rc, lc);
            ok = (trig_req_ != nullptr);
        }
        gpiod_request_config_free(rc);
        gpiod_line_config_free(lc);
        gpiod_line_settings_free(s);
        if (!ok) return false;

        // Echo: 输入, 双边沿, 下拉
        struct gpiod_line_settings* s2 = gpiod_line_settings_new();
        if (!s2) return false;
        gpiod_line_settings_set_direction(s2, GPIOD_LINE_DIRECTION_INPUT);
        gpiod_line_settings_set_edge_detection(s2, GPIOD_LINE_EDGE_BOTH);
        gpiod_line_settings_set_bias(s2, GPIOD_LINE_BIAS_PULL_DOWN);
        size_t off2 = echo_off_[i];
        struct gpiod_line_config* lc2 = gpiod_line_config_new();
        if (!lc2) { gpiod_line_settings_free(s2); return false; }
        ok = (gpiod_line_config_add_line_settings(lc2, &off2, 1, s2) == 0);
        struct gpiod_request_config* rc2 = gpiod_request_config_new();
        if (!rc2) { gpiod_line_config_free(lc2); gpiod_line_settings_free(s2); return false; }
        gpiod_request_config_set_consumer(rc2, "ultrasonic_node");
        if (ok) {
            echo_req_ = gpiod_chip_request_lines(chip_, rc2, lc2);
            ok = (echo_req_ != nullptr);
        }
        gpiod_request_config_free(rc2);
        gpiod_line_config_free(lc2);
        gpiod_line_settings_free(s2);
        if (!ok) return false;
        if (!ev_buf_) ev_buf_ = gpiod_edge_event_buffer_new(8);
        return ev_buf_ != nullptr;
    }

    void set_trig_(size_t i, bool high) {
        if (trig_req_) {
            gpiod_line_request_set_value(trig_req_, trig_off_[i],
                high ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
        }
    }

    bool wait_edge_(size_t i, bool rising, uint64_t& ts_ns) {
        if (!echo_req_ || !ev_buf_) return false;
        long tmo_ns = (long)kEchoTimeoutMs * 1000 * 1000;
        int rc = gpiod_line_request_wait_edge_events(echo_req_, tmo_ns);
        if (rc <= 0) return false;
        int nev = gpiod_line_request_read_edge_events(echo_req_, ev_buf_, 8);
        if (nev <= 0) return false;
        for (int k = 0; k < nev; ++k) {
            struct gpiod_edge_event* ev = gpiod_edge_event_buffer_get_event(ev_buf_, (unsigned long)k);
            if (!ev) continue;
            if (gpiod_edge_event_get_line_offset(ev) != echo_off_[i]) continue;
            bool is_rise = (gpiod_edge_event_get_event_type(ev) == GPIOD_EDGE_EVENT_RISING_EDGE);
            if (is_rise == rising) {
                ts_ns = gpiod_edge_event_get_timestamp_ns(ev);
                return true;
            }
        }
        return false;
    }
#else  // v1 (libgpiod 1.x)
    void set_trig_(size_t i, bool high) {
        if (trig_line_[i]) gpiod_line_set_value(trig_line_[i], high ? 1 : 0);
    }

    bool wait_edge_(size_t i, bool rising, uint64_t& ts_ns) {
        struct timespec tmo;
        tmo.tv_sec  = 0;
        tmo.tv_nsec = (long)kEchoTimeoutMs * 1000 * 1000;
        if (gpiod_line_event_wait(echo_line_[i], &tmo) != 1) return false;
        struct gpiod_line_event ev;
        if (gpiod_line_event_read(echo_line_[i], &ev) != 0) return false;
        ts_ns = (uint64_t)ev.ts.tv_sec * 1000000000ull + (uint64_t)ev.ts.tv_nsec;
        bool is_rise = (ev.event_type == GPIOD_LINE_EVENT_RISING_EDGE);
        return (is_rise == rising);
    }
    struct gpiod_line* trig_line_[kMaxChannels] = {};
    struct gpiod_line* echo_line_[kMaxChannels] = {};
#endif

    struct gpiod_chip* chip_ = nullptr;
    std::string chip_path_;
    std::string chip_label_;
    bool line_ok_[kMaxChannels] = {};
    unsigned trig_off_[kMaxChannels] = {};
    unsigned echo_off_[kMaxChannels] = {};
#ifdef LIBGPIOD_V2
    struct gpiod_line_request* trig_req_ = nullptr;
    struct gpiod_line_request* echo_req_ = nullptr;
    struct gpiod_edge_event_buffer* ev_buf_ = nullptr;
#endif
};
#endif  // USE_GPIO

class UltrasonicNode : public rclcpp::Node {
public:
    UltrasonicNode() : Node("ultrasonic_node") {
        // ---- 参数 ----
        use_gpio_  = this->declare_parameter("use_gpio", false);
        rate_hz_   = this->declare_parameter("rate_hz", 10.0);
        // ROS2 整数数组参数默认 int64 vector; 声明为 vector<int64_t> 避免类型不匹配
        trig_pins_ = this->declare_parameter("trig_pins", std::vector<int64_t>{23, 17, 5, 13});
        echo_pins_ = this->declare_parameter("echo_pins", std::vector<int64_t>{24, 27, 6, 19});
        gpio_chip_ = this->declare_parameter("gpio_chip", std::string("auto"));
        std::vector<int64_t> active = this->declare_parameter("active", std::vector<int64_t>{0, 1, 2, 3});

        // active 清洗: 只留 [0,3], 去重; 非法值警告并忽略
        for (auto v : active) {
            if (v < 0 || v >= kMaxChannels) {
                RCLCPP_WARN(get_logger(), "active 含非法下标 %ld (有效 0-3), 已忽略", (long)v);
                continue;
            }
            bool dup = false;
            for (int x : active_idx_) if (x == (int)v) dup = true;
            if (!dup) active_idx_.push_back((int)v);
        }

        // ---- 发布器 ----
        pub_ = this->create_publisher<UltraMsg>("/ultrasonic", rclcpp::SensorDataQoS());

        // ---- 定时器 (默认 10Hz) ----
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / std::max(rate_hz_, 1.0)),
            [this]() { publish_ultrasonic(); });

#ifdef USE_GPIO
        if (use_gpio_) {
            reader_ = std::make_unique<Hcsr04Reader>(trig_pins_, echo_pins_, gpio_chip_);
            if (!reader_->chip_open()) {
                RCLCPP_ERROR(get_logger(),
                    "GPIO 初始化失败 (gpio_chip=%s) ⇒ 全部通道发布无效读数 (fail-closed)",
                    gpio_chip_.c_str());
            } else {
                RCLCPP_INFO(get_logger(), "GPIO 真读已启用: chip=%s label='%s' (active=%zu 路)",
                            reader_->chip_path().c_str(), reader_->chip_label().c_str(),
                            active_idx_.size());
            }
        }
#else
        if (use_gpio_) {
            RCLCPP_WARN(get_logger(),
                "本构建未启用 USE_GPIO (需 colcon --cmake-args -DUSE_GPIO=ON), use_gpio 被忽略, 走模拟");
        }
#endif

        RCLCPP_INFO(get_logger(), "ultrasonic_node 启动 (use_gpio=%d, rate=%.1fHz)",
                    use_gpio_ ? 1 : 0, rate_hz_);
    }

private:
    void publish_ultrasonic() {
        UltraMsg msg;
        msg.stamp = get_clock()->now();
        msg.seq = seq_++;
        msg.period_sec = static_cast<float>(1.0 / std::max(rate_hz_, 1.0));

        // 初值: 全部无效 (fail-closed; 接入的通道在下面逐路覆盖)
        msg.front_left_cm = msg.front_center_cm = msg.front_right_cm = msg.bottom_cm =
            std::numeric_limits<double>::quiet_NaN();
        msg.front_left_valid = msg.front_center_valid = msg.front_right_valid =
            msg.bottom_valid = false;

        const std::array<double*, kMaxChannels> dist{
            &msg.front_left_cm, &msg.front_center_cm, &msg.front_right_cm, &msg.bottom_cm};
        const std::array<bool*, kMaxChannels> val{
            &msg.front_left_valid, &msg.front_center_valid, &msg.front_right_valid, &msg.bottom_valid};

        bool real = false;
#ifdef USE_GPIO
        if (use_gpio_ && reader_ && reader_->chip_open()) {
            real = true;
            size_t fail = 0;
            for (int i : active_idx_) {
                double cm = reader_->measure((size_t)i);
                *dist[i] = cm;
                *val[i] = std::isfinite(cm);
                if (!std::isfinite(cm)) ++fail;
            }
            if (!active_idx_.empty() && fail == active_idx_.size()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "超声: 本周期 active 通道全部无效 (接线/分压/供电/量程?)");
            } else {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "超声: FL=%.1f FC=%.1f FR=%.1f BOT=%.1f (cm; nan=无效/未启用)",
                    *dist[0], *dist[1], *dist[2], *dist[3]);
            }
        }
#endif
        if (!real) {
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
    std::string gpio_chip_ = "auto";
    std::vector<int> active_idx_;
#ifdef USE_GPIO
    std::unique_ptr<Hcsr04Reader> reader_;
#endif
    std::mt19937 gen_{static_cast<unsigned>(std::random_device{}())};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UltrasonicNode>());
    rclcpp::shutdown();
    return 0;
}
