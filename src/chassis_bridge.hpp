/**
 * 底盘通信抽象层 (ChassisBridge)
 *
 * 职责: 把速度指令 (linear, angular) 发送到机械狗底盘。
 * 设计: 可插拔接口 —— 换底盘通信方式只需换一个实现类, 节点代码零改动。
 *
 * 实现:
 *   - SimulatedChassisBridge (默认): 打印日志, 无硬件依赖, PC 模拟跑通链路
 *   - Stm32ChassisBridge (占位): 待确认 2×STM32 通信方式 (FDCAN/串口) 后实现
 *
 * 使用 (chassis_bridge_node):
 *   ros2 run mechdog_navigation_ros chassis_bridge_node --ros-args -p bridge_type:=simulated
 *   # 真机: bridge_type:=stm32
 */
#pragma once

#include <iostream>
#include <memory>
#include <string>

namespace mechdog_ros {

/**
 * 底盘通信抽象接口
 * 所有底盘实现只需实现 send_velocity, 接收线速度(m/s)与角速度(rad/s)。
 */
class ChassisBridge {
public:
    virtual ~ChassisBridge() = default;

    /** 发送速度指令到底盘 */
    virtual void send_velocity(double linear, double angular) = 0;

    /** 按名称创建桥接实现 (simulated / stm32) */
    static std::unique_ptr<ChassisBridge> create(const std::string& type);
};

/**
 * 模拟实现 (默认): 打印日志, 验证链路用, 无硬件依赖。
 */
class SimulatedChassisBridge : public ChassisBridge {
public:
    void send_velocity(double linear, double angular) override {
        std::cout << "[ChassisBridge:simulated] vx=" << linear
                  << " m/s, wz=" << angular << " rad/s" << std::endl;
    }
};

/**
 * STM32 真机实现 (占位骨架)
 *
 * 待确认 (回学校后):
 *   1. 底盘通信方式: FDCAN? 串口? (第二个 STM32 的用途决定)
 *   2. 指令协议: 直接 vx/wz? 还是转角+速度?
 *   确认后在本类实现 send_velocity, 其余代码无需改动。
 */
class Stm32ChassisBridge : public ChassisBridge {
public:
    Stm32ChassisBridge() {
        // TODO(硬件确认后): 初始化 FDCAN/串口 通信
    }
    void send_velocity(double linear, double angular) override {
        // TODO(硬件确认后): 将 (linear, angular) 编码发送到 STM32
        (void)linear;
        (void)angular;
        std::cerr << "[ChassisBridge:stm32] 未实现: 待确认底盘通信方式" << std::endl;
    }
};

inline std::unique_ptr<ChassisBridge> ChassisBridge::create(const std::string& type) {
    if (type == "stm32") {
        return std::make_unique<Stm32ChassisBridge>();
    }
    // 默认 simulated
    return std::make_unique<SimulatedChassisBridge>();
}

} // namespace mechdog_ros
