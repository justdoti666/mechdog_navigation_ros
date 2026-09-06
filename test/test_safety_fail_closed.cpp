/**
 * REVIEW R5 集成测试: v2.3 fail-closed 语义跨层验证
 *
 * 背景 (见 ../docs/REVIEW_FIXPLAN.md R5): 算法库 v2.3 引入语义级 fail-closed 决策 ——
 *   全传感器无效时 determine_action 必须 STOP (M1), 前向全盲降速 (R3),
 *   min_forward_distance_m==8.0 仅为"无有效数据"兜底值, 不得被消费方当作"前方开阔"而 FORWARD。
 *   若 ROS 侧 safety_node 自行 reinterpret min_forward_distance_m 或忽略 sensor 有效性,
 *   会在新语义下重现 fail-open —— 本测试固定 safety_node 的真实消费路径:
 *   fusion_->fuse() → planner_->plan(result) → (linear, angular)。
 *
 * 通过标准: 注入"全传感器无效" (相机无有效像素 + 超声全 invalid 经方案A注入) →
 *   recommended_action==STOP, 且 plan() 输出 (0.0, 0.0)；反之若有人改回
 *   "8.0m 视为开阔" 的 fail-open 解读, 此处立刻红。
 */
#include <gtest/gtest.h>

#include "sensor_astra.h"
#include "sensor_ultrasonic.h"
#include "sensor_ir.h"
#include "sensor_fusion.h"
#include "path_planner.h"

using namespace mechdog;

// 全传感器无效帧 (方案A 注入) 贯穿 fuse() → plan() 全链路, 必须 STOP + 零速
TEST(SafetyFailClosed, AllInvalidInjected_stopsOnFullChain) {
    AstraProDriver astra(true);           // 不 start(): latest_frame_ 保持默认 invalid (模拟相机无输出)
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    // 方案A: 注入 ultrasonic_node 上报的"全无效"数据 (stale 前 read_all 直接用注入缓存)
    UltrasonicArrayData d;
    d.timestamp = 0.0;
    d.front_left.valid   = false; d.front_left.distance_cm   = UltrasonicConfig::kUltrasonicInvalidM * 100;
    d.front_center.valid = false; d.front_center.distance_cm = UltrasonicConfig::kUltrasonicInvalidM * 100;
    d.front_right.valid  = false; d.front_right.distance_cm  = UltrasonicConfig::kUltrasonicInvalidM * 100;
    d.bottom.valid       = false; d.bottom.distance_cm       = UltrasonicConfig::kUltrasonicInvalidM * 100;
    ultrasonic.inject_external_data(d);

    auto result = fusion.fuse();

    // M1 跨层断言: 全失效 → sensors_valid=false → determine_action STOP
    ASSERT_FALSE(result.sensors_valid);
    EXPECT_EQ(result.recommended_action, NavigationAction::STOP);
    // 兜底值必须保留 8.0 (供可视化), 但决策不得依赖它 —— 消费方不得 reinterpret 为"开阔"
    EXPECT_DOUBLE_EQ(result.min_forward_distance_m, 8.0);

    // safety_node 消费路径 (新鲜 planner, last=0): STOP → 零速
    PathPlanner planner;
    VelocityCmd cmd = planner.plan(result);
    EXPECT_DOUBLE_EQ(cmd.linear, 0.0);
    EXPECT_DOUBLE_EQ(cmd.angular, 0.0);
}

// 对照组: min_fwd==8.0 但是 STOP 语义的 FusionResult, 经 plan() 也绝不产生非零速度
// (若有人在 ROS 层改判"8.0=开阔"会在此暴露)
TEST(SafetyFailClosed, FallbackEightMetersNeverBecomesForward) {
    FusionResult result;
    result.min_forward_distance_m = 8.0;   // 兜底值 (全失效)
    result.recommended_action = NavigationAction::STOP;  // 算法库已判 STOP (M1)

    PathPlanner planner;
    VelocityCmd cmd = planner.plan(result);
    EXPECT_DOUBLE_EQ(cmd.linear, 0.0);
    EXPECT_DOUBLE_EQ(cmd.angular, 0.0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
