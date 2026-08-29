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
 *   # 可选参数: serial_port:=/dev/ttyACM0 baudrate:=115200
 *
 * 注意: 若师兄 quadruped_ws 已运行 wheel_board_bridge_node (它直接管串口),
 *       本节点应保持 bridge_type:=simulated, 避免双写串口。
 */
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "chassis_bridge.hpp"

using namespace mechdog_ros;

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

        // 订阅 /cmd_vel: 收到速度指令 → 发送到底盘
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                bridge_->send_velocity(msg->linear.x, msg->angular.z);
            });

        RCLCPP_INFO(this->get_logger(), "chassis_bridge_node 启动: bridge_type=%s",
            bridge_type.c_str());
    }

private:
    std::unique_ptr<ChassisBridge> bridge_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
