// test_ultra_publish_mode.cpp — U-1 publish-mode seam 单测 (先红后绿, 2026-10-05)
//
// 锁定三态判定: (use_gpio, gpio_compiled, chip_open) -> PublishMode
// T1-T6 与修复方案 §三.U-1 用例表逐条对应 (6 例 = 6 个 TEST, 便于 colcon 计数)。
#include <gtest/gtest.h>

#include "ultra_publish_mode.hpp"

using mechdog_ultra::PublishMode;
using mechdog_ultra::resolve_publish_mode;

// T1: 默认 (未要求真读) => 模拟 (PC/WSL 链路行为, 不变)
TEST(UltraPublishMode, T1_SimulatedByDefault) {
    EXPECT_EQ(resolve_publish_mode(false, false, false), PublishMode::Simulated);
}

// T2: 未要求真读, 即使环境可用也仍走模拟 => 模拟
TEST(UltraPublishMode, T2_SimulatedWhenNotRequested) {
    EXPECT_EQ(resolve_publish_mode(false, true, true), PublishMode::Simulated);
}

// T3: 要求真读但二进制未编入 USE_GPIO => 全无效 (fail-closed, 绝不模拟)
TEST(UltraPublishMode, T3_AllInvalidWhenNotCompiled) {
    EXPECT_EQ(resolve_publish_mode(true, false, false), PublishMode::AllInvalid);
}

// T4: 要求真读且已编译, 但 gpiochip 打不开 => 全无效 (fail-closed, 绝不模拟)
TEST(UltraPublishMode, T4_AllInvalidWhenChipOpenFailed) {
    EXPECT_EQ(resolve_publish_mode(true, true, false), PublishMode::AllInvalid);
}

// T5: 矛盾输入 (说"芯片已开"但根本没编译) 也按保守处理 => 全无效
TEST(UltraPublishMode, T5_AllInvalidOnContradictoryInput) {
    EXPECT_EQ(resolve_publish_mode(true, false, true), PublishMode::AllInvalid);
}

// T6: 要求真读且环境齐备 => 真读
TEST(UltraPublishMode, T6_RealGpioWhenAvailable) {
    EXPECT_EQ(resolve_publish_mode(true, true, true), PublishMode::RealGpio);
}
