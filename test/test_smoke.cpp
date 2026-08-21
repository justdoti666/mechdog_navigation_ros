/**
 * ROS-10 (v2.2) smoke test —— 验证 chassis_bridge.hpp 链接 + 基本构造
 * (纯 C++ 头, 无 rclcpp 依赖; 在 ROS 构建/Linux 上由 ament_add_gtest 驱动)
 */
#include <gtest/gtest.h>

#include "chassis_bridge.hpp"

// 模拟桥可构造 + send_velocity 不崩
TEST(Smoke, SimulatedBridgeConstructsAndSends) {
    mechdog_ros::SimulatedChassisBridge bridge;
    bridge.send_velocity(0.1, 0.0);   // 节流后仍调用, 不抛即通过
    bridge.send_velocity(0.2, 0.3);
    SUCCEED();
}

// 抽象接口可指向具体实现 (验证 vtable 链接)
TEST(Smoke, PolymorphicDispatch) {
    std::unique_ptr<mechdog_ros::ChassisBridge> bridge =
        std::make_unique<mechdog_ros::SimulatedChassisBridge>();
    bridge->send_velocity(0.0, 0.0);
    SUCCEED();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
