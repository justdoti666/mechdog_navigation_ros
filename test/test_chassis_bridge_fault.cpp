/**
 * v2.9.21 (T-B4, 复审批 B12): 底盘桥串口故障语义 —— 离线测试。
 *
 * 断言:
 *   1. 串口未打开 (不存在的设备路径 ⇒ fd=-1) 时, 连续 3 条指令未送达 ⇒ fault() 置位;
 *   2. 故障后继续调用不崩溃、状态保持 (接口语义: 不停止发布, 恢复自动解除);
 *   3. SimulatedChassisBridge fault() 恒 false (基类默认实现)。
 *
 * 注: "写失败" 分支与未打开分支共用同一故障链逻辑 (fail_streak_>=3 ⇒ fault);
 *     真实写失败需注入 pty 才可离线构造, 此处覆盖 fd<0 路径 + 接口语义。
 */
#include <gtest/gtest.h>

#include "chassis_bridge.hpp"
#include "cmd_vel_watchdog.hpp"   // v2.9.23 (批B B12): 上游静默判据

using namespace mechdog_ros;

// 本仓约定: 各测试自带 main (SKIP_LINKING_MAIN_LIBRARIES, 见 CMakeLists v2.9.18 A3 注)
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(ChassisBridgeFault, NotOpenFaultsAfterThree) {
    Stm32ChassisBridge b("/dev/nonexistent_mdw_test_port", 115200);
    EXPECT_FALSE(b.fault());          // 尚未发过指令
    b.send_velocity(0.10, 0.0);
    b.send_velocity(0.10, 0.0);
    EXPECT_FALSE(b.fault());          // 未达 3 条
    b.send_velocity(0.10, 0.0);
    EXPECT_TRUE(b.fault());           // 连续 3 条未送达 ⇒ 链路故障
    b.send_velocity(0.00, 0.0);       // 故障后继续发布: 不崩溃
    EXPECT_TRUE(b.fault());
}

TEST(ChassisBridgeFault, SimulatedNeverFaults) {
    SimulatedChassisBridge s;
    s.send_velocity(0.20, 0.10);
    s.send_velocity(0.00, 0.00);
    EXPECT_FALSE(s.fault());
}

// v2.9.23 (批B B12 残, 二轮审查): 上游静默判据边界 —— 严格 > timeout 才触发;
//   未设起点 (last<=0) 不判 (节点层用构造时刻做起点 ⇒ "启动→首指令"静默同样受保护)。
TEST(UpstreamWatchdog, SilenceBoundary) {
    const int64_t to = 500LL * 1000000;        // 500ms (ns)
    const int64_t t0 = 1000000000LL;
    EXPECT_FALSE(upstream_silent(0, t0, to));                  // 未设起点: 不判 (防御)
    EXPECT_FALSE(upstream_silent(t0, t0 + to, to));            // 恰好 = timeout: 未超
    EXPECT_FALSE(upstream_silent(t0, t0 + to - 1, to));        // 差 1ns: 未超
    EXPECT_TRUE (upstream_silent(t0, t0 + to + 1, to));        // 超 1ns: 静默
    EXPECT_TRUE (upstream_silent(t0, t0 + 10 * to, to));       // 长时间静默
}
