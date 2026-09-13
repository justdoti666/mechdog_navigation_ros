// v2.6: 相机几何 → 地面高度先验 单测。
// 锁住"台架 0.6m 与装机 0.2m 只差一个参数"这条口径, 防止再把装机值写死进台架实验。
#include <gtest/gtest.h>

#include "sensor_geometry.hpp"

using mechdog_ros::derived_ground_prior_z;

// 台架: base 原点=相机 (E.z=0), 相机离地 0.6 → 地面在 base 系 -0.6
TEST(SensorGeometry, BenchSetupCameraAtZeroPointSix) {
    EXPECT_NEAR(derived_ground_prior_z(0.6, 0.0), -0.6, 1e-9);
}

// 装机: 相机离地 0.2, 外参 z 按量测 (base 原点在相机下方 0.18 时 → 地面 -0.02)
TEST(SensorGeometry, InstalledVariantDependsOnExtrinsicZ) {
    EXPECT_NEAR(derived_ground_prior_z(0.2, 0.18), -0.02, 1e-9);
    // base 原点与相机同高 (E.z=0) 时, 地面高度 = -相机离地高
    EXPECT_NEAR(derived_ground_prior_z(0.2, 0.0), -0.2, 1e-9);
    // 装机默认那组常数 (0.18/0.18) 推出 0.0 —— 与仓库当前先验 -0.18 不一致 (0.18m 口径差),
    // 这是历史常数的自洽问题, 需要实测量测后二选一定案 (记在 PROGRESS 4e/4f)。
    EXPECT_NEAR(derived_ground_prior_z(0.18, 0.18), 0.0, 1e-9);
}

// 单调性: 相机越高, 地面在 base 系越低 (越负)
TEST(SensorGeometry, MonotonicInCameraHeight) {
    EXPECT_LT(derived_ground_prior_z(0.6, 0.0), derived_ground_prior_z(0.2, 0.0));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
