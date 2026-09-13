// v2.5: 超声来源解析 单测 —— 锁住"真机上绝不静默使用模拟随机数"这条安全不变式。
// 背景: Pi 上未编 USE_WIRINGPI → 超声驱动是编译期模拟器, 数据带 valid=true 且底部
// 有 5% 概率模拟悬崖 → 实测 245 帧里 14 帧 cliff=YES 的假 STOP (见 artifacts 2026-09-13)。
#include <gtest/gtest.h>

#include "ultrasonic_source.hpp"

using mechdog_ros::resolve_ultrasonic_source;

// auto: 按可用性递补, 都没有 → none (绝不回落到 simulated)
TEST(UltrasonicSource, AutoPrefersRealSourcesAndNeverFallsBackToSimulated) {
    // 整机模拟模式 → simulated
    EXPECT_EQ(resolve_ultrasonic_source("auto", true, false, 0, false), "simulated");
    // 真机 + 有 /ultrasonic 发布者 → topic
    EXPECT_EQ(resolve_ultrasonic_source("auto", false, false, 1, false), "topic");
    // 真机 + 只有 GPIO → hardware
    EXPECT_EQ(resolve_ultrasonic_source("auto", false, true, 0, false), "hardware");
    // 真机 + 什么都没有 → none  ← 关键: 不是 simulated
    EXPECT_EQ(resolve_ultrasonic_source("auto", false, false, 0, false), "none");
    // topic 优先于 hardware (话题是底盘板/桥的数据源)
    EXPECT_EQ(resolve_ultrasonic_source("auto", false, true, 2, false), "topic");
}

// 显式请求: 只有 simulated 且(模拟模式 或 显式允许)才给 simulated
TEST(UltrasonicSource, ExplicitSimulatedRequiresOptIn) {
    EXPECT_EQ(resolve_ultrasonic_source("simulated", false, false, 0, false), "none");
    EXPECT_EQ(resolve_ultrasonic_source("simulated", false, false, 0, true), "simulated");
    EXPECT_EQ(resolve_ultrasonic_source("simulated", true, false, 0, false), "simulated");
}

// hardware 请求但 GPIO 不可用 → none (不静默换成模拟)
TEST(UltrasonicSource, HardwareRequestedWithoutGpioBecomesNone) {
    EXPECT_EQ(resolve_ultrasonic_source("hardware", false, false, 0, false), "none");
    EXPECT_EQ(resolve_ultrasonic_source("hardware", false, true, 0, false), "hardware");
}

// 显式 none / topic 原样保留 (topic 即使此刻无发布者也保持 topic,
// 由节点在超时后禁用 —— 超声数据可能是晚到的)
TEST(UltrasonicSource, ExplicitNoneAndTopicAreHonored) {
    EXPECT_EQ(resolve_ultrasonic_source("none", false, true, 3, true), "none");
    EXPECT_EQ(resolve_ultrasonic_source("topic", false, false, 0, false), "topic");
}

// 未知取值按 auto 处理 (保守: 宁可 none 也不 simulated)
TEST(UltrasonicSource, UnknownValueBehavesLikeAuto) {
    EXPECT_EQ(resolve_ultrasonic_source("garbage", false, false, 0, false), "none");
    EXPECT_EQ(resolve_ultrasonic_source("garbage", true, false, 0, false), "simulated");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
