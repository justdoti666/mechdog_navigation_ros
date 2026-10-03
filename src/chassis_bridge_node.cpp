/**
 * chassis_bridge_node: 底盘通信节点
 *
 * 职责: 订阅 /cmd_vel (Twist), 通过 ChassisBridge 接口发送到底盘。
 *       bridge_type 参数切换实现: simulated (默认) / stm32 (串口, 21字节帧)。
 *
 * 运行:
 *   ros2 run mechdog_navigation_ros chassis_bridge_node            # 模拟
 *   ros2 run mechdog_navigation_ros chassis_bridge_node \
 *       --ros-args -p bridge_type:=stm32                           # 真机(串口)
 *   # 可选参数: serial_port:=/dev/ttyACM0 baudrate:=115200 cmd_vel_timeout_ms:=500
 *
 * 注意: 若师兄 quadruped_ws 已运行 wheel_board_bridge_node (它直接管串口),
 *       本节点应保持 bridge_type:=simulated, 避免双写串口。
 *
 * v2.9.23 (批B B12 残, 二轮审查): 上游看门狗 —— /cmd_vel 静默超 cmd_vel_timeout_ms
 *   (默认 500ms) ⇒ 告警 + 周期性零速兜底帧。防"上游(闸门/safety_node)崩溃或静默时,
 *   STM32 永久保持最后一条指令"。发送失败兜底另见 T-B4 (v2.9.21)。
 *   计时起点 = 节点启动 (启动→首条指令间的静默同样受保护, 底盘保持静止等待上游)。
 */
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "chassis_bridge.hpp"
#include "cmd_vel_watchdog.hpp"

using namespace mechdog_ros;

namespace {
int64_t steady_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class ChassisBridgeNode : public rclcpp::Node {
public:
    ChassisBridgeNode() : Node("chassis_bridge_node") {
        // 参数: bridge_type (simulated 默认 / stm32 真机)
        auto bridge_type = this->declare_parameter<std::string>("bridge_type", "simulated");

        if (bridge_type == "stm32") {
            auto port = this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
            auto baud = this->declare_parameter<int>("baudrate", 115200);
            bridge_ = std::make_unique<Stm32ChassisBridge>(port, baud);
        } else {
            bridge_ = std::make_unique<SimulatedChassisBridge>();
        }

        // v2.9.23 (批B B12 残): 上游看门狗参数 + 计时起点 (节点启动即开始计时)
        cmd_vel_timeout_ms_ = std::max(100, static_cast<int>(
            this->declare_parameter("cmd_vel_timeout_ms", 500)));
        last_cmd_ns_ = steady_now_ns();

        // 订阅 /cmd_vel: 收到速度指令 → 发送到底盘
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                last_cmd_ns_ = steady_now_ns();
                ever_cmd_ = true;
                if (upstream_silent_) {   // 上游恢复: 解除静默, 恢复透传
                    upstream_silent_ = false;
                    RCLCPP_INFO(this->get_logger(),
                        "上游 /cmd_vel 恢复到达 —— 退出零速兜底, 恢复透传");
                }
                bridge_->send_velocity(msg->linear.x, msg->angular.z);
                // v2.9.21 (T-B4): 链路故障上报 (一次 WARN; 桥内部已尝试零速兜底帧)
                if (bridge_->fault() && !fault_warned_) {
                    fault_warned_ = true;
                    RCLCPP_WARN(this->get_logger(),
                        "底盘链路故障: 连续发送失败 -- 指令可能未到达底盘 (桥已尝试零速兜底帧)");
                }
            });

        // v2.9.23 (批B B12 残): 100ms 周期检查上游静默 (单线程 executor: 与订阅回调同线程)
        watchdog_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() { upstream_watchdog_tick(); });

        RCLCPP_INFO(this->get_logger(),
            "chassis_bridge_node 启动: bridge_type=%s (上游看门狗: %d ms)",
            bridge_type.c_str(), cmd_vel_timeout_ms_);
    }

private:
    // v2.9.23 (批B B12 残): 静默超时 ⇒ 告警一次 + 周期性零速兜底帧 (幂等)。
    //   判据是纯函数 (cmd_vel_watchdog.hpp), 边界语义由单测锁定: 严格 > timeout 才触发。
    void upstream_watchdog_tick() {
        const int64_t now_ns = steady_now_ns();
        const int64_t timeout_ns = static_cast<int64_t>(cmd_vel_timeout_ms_) * 1000000LL;
        if (!mechdog_ros::upstream_silent(last_cmd_ns_, now_ns, timeout_ns)) {
            return;
        }
        if (!upstream_silent_) {
            upstream_silent_ = true;
            if (ever_cmd_) {
                RCLCPP_WARN(this->get_logger(),
                    "上游 /cmd_vel 已静默 > %d ms —— 主动发零速兜底帧 "
                    "(防 STM32 永久保持最后指令; 上游恢复后自动退出)",
                    cmd_vel_timeout_ms_);
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "启动后 %d ms 内未收到任何 /cmd_vel —— 开始发零速兜底帧 "
                    "(底盘保持静止, 直到上游发指令)",
                    cmd_vel_timeout_ms_);
            }
        }
        // 每 timeout 周期重发零速 (幂等; 静默期间持续保持底盘静止)
        if (last_zero_ns_ == 0 || now_ns - last_zero_ns_ >= timeout_ns) {
            bridge_->send_velocity(0.0, 0.0);
            last_zero_ns_ = now_ns;
        }
    }

    std::unique_ptr<ChassisBridge> bridge_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    bool fault_warned_ = false;   // v2.9.21 (T-B4): 链路故障 WARN 只发一次
    // v2.9.23 (批B B12 残): 上游看门狗状态
    rclcpp::TimerBase::SharedPtr watchdog_;
    int cmd_vel_timeout_ms_ = 500;
    int64_t last_cmd_ns_ = 0;      // 最后一条 /cmd_vel 时刻 (steady ns; 构造时=启动时刻)
    int64_t last_zero_ns_ = 0;     // 上次零速兜底帧时刻 (0=还没发过)
    bool upstream_silent_ = false; // 当前是否处于静默兜底状态
    bool ever_cmd_ = false;        // 是否收到过指令 (区分两种告警文案)
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
