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
#include "safety_warmup.hpp"   // R4 (REVIEW): 启动预热等待

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

// R4 (REVIEW): 启动预热 —— 等底部线程产出首帧, 消除启动期 is_fall_risk() 的一帧误急停。
// sim 驱动 bottom 线程 20Hz (~50-150ms 首帧); 2s 预算下 wait_for_bottom_ready 必然返回 true。
TEST(SafetyFailClosed, WarmupWaitsForBottomReady) {
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    bool ready = mechdog_ros::wait_for_bottom_ready(
        ultrasonic, std::chrono::milliseconds(2000));
    EXPECT_TRUE(ready);   // 若 sim bottom 线程异常未产出, 此处失败即暴露
}

// v2.4: 外部注入深度帧 (depth_source=topic) —— 无 Astra SDK 的机器 (如 Pi 5B) 靠它把
//   真深度 (ros2_astra_camera 的话题) 送进融合/点云/2.5D/路1, 全程不依赖 SDK。
TEST(DepthTopicSource, InjectedFrameDrivesFusionAndFailsClosed) {
    AstraProDriver astra(false);       // 真机模式但**不 start()** (topic 源由话题驱动)
    UltrasonicArrayDriver ultrasonic(get_ultrasonic_layout());
    InfraRedSensor ir(true);
    SensorFusion fusion(&astra, &ultrasonic, &ir);

    // 1) 未注入 → 帧无效 (fail-closed)
    EXPECT_FALSE(astra.get_latest_frame().valid);

    // 2) 注入 1.0m 的"墙" → 帧有效 + 区域分析给出 ~1.0m
    std::vector<uint16_t> wall(640 * 480, 1000);
    ASSERT_TRUE(astra.inject_depth_frame(wall, 640, 480, 100.0));
    auto f = astra.get_latest_frame();
    ASSERT_TRUE(f.valid);
    EXPECT_EQ(f.depth_width, 640);
    EXPECT_EQ(f.depth_height, 480);
    EXPECT_GT(f.center_region.valid_pixel_ratio, 0.9);
    EXPECT_NEAR(f.center_region.min_distance_m, 1.0, 0.05);

    // 3) 值域过滤与 capture_real 同口径: 10cm (<600mm) → 全是无效像素
    std::vector<uint16_t> too_near(640 * 480, 100);
    ASSERT_TRUE(astra.inject_depth_frame(too_near, 640, 480, 101.0));
    EXPECT_DOUBLE_EQ(astra.get_latest_frame().center_region.valid_pixel_ratio, 0.0);

    // 4) 尺寸/长度非法 → 拒绝 (原帧保持不动, 不污染)
    EXPECT_FALSE(astra.inject_depth_frame(too_near, 0, 480, 102.0));
    EXPECT_FALSE(astra.inject_depth_frame(std::vector<uint16_t>(10, 1000), 640, 480, 102.0));

    // 5) 有效注入 + 前向超声全无效 → 融合必须看到 ~1m 前向障碍 (不是 8.0m 兜底)
    //    注意: bottom 必须注入"有效且贴地" —— 若连 bottom 也注无效, is_fall_risk()
    //    会按 fail-closed 判"有风险" → cliff_detected=true → STOP (悬崖层的正确行为,
    //    但那不是本用例要验的东西)。
    ASSERT_TRUE(astra.inject_depth_frame(wall, 640, 480, 103.0));
    UltrasonicArrayData ud;
    ud.timestamp = 0.0;
    ud.front_left.valid = ud.front_center.valid = ud.front_right.valid = false;
    const double inv_cm = UltrasonicConfig::kUltrasonicInvalidM * 100;
    ud.front_left.distance_cm = ud.front_center.distance_cm = inv_cm;
    ud.front_right.distance_cm = inv_cm;
    ud.bottom.valid = true;
    ud.bottom.distance_cm = 15.0;            // 贴地 (<=30cm) → 无悬崖风险
    ultrasonic.inject_external_data(ud);

    auto r = fusion.fuse();
    EXPECT_TRUE(r.sensors_valid);
    EXPECT_FALSE(r.cliff_detected);
    EXPECT_NEAR(r.obstacles.at("center").astra_dist_m, 1.0, 0.05);
    EXPECT_LT(r.min_forward_distance_m, 1.2);
    EXPECT_EQ(r.recommended_action, NavigationAction::FORWARD);  // 1m 开阔 → 前进 (非 STOP)

    // 6) 源断开 (话题超时) → invalidate_frame → 深度退出融合 → 全失效 STOP (fail-closed)
    astra.invalidate_frame();
    EXPECT_FALSE(astra.get_latest_frame().valid);
    UltrasonicArrayData all_bad = ud;
    all_bad.front_left.valid = all_bad.front_center.valid = all_bad.front_right.valid = false;
    all_bad.bottom.valid = false;            // bottom 也断 → fail-closed
    all_bad.bottom.distance_cm = inv_cm;
    ultrasonic.inject_external_data(all_bad);
    auto r2 = fusion.fuse();
    EXPECT_FALSE(r2.sensors_valid);
    EXPECT_EQ(r2.recommended_action, NavigationAction::STOP);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
