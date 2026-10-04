// S2 (2026-10-04): 雷达扇区消费 离线首版 单测 —— 锁定三条安全语义:
//   ① 扇区判定: L=[lo,lc) · C=[lc,rc) · R=[rc,hi] (左闭右开, 最右端闭合)
//   ② 无效点过滤: NaN/inf/0.0/超量程 一律跳过 (既不当"近障碍"也不当"畅通")
//   ③ fail-closed: 扇区无有效点 ⇒ NaN (绝不给 0/默认值); 配置非法 ⇒ 整体全 NaN
// 用例期望值均按 IEEE 双精度实际落点核对过 (核对脚本存证据目录);
// 边界用例统一用 ±0.001° 偏移, 避开名义分界值的浮点歧义 (语义 = 左闭右开的自然延伸)。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "lidar_sectors.hpp"

using mechdog_ros::LidarSectorConfig;
using mechdog_ros::compute_lidar_sectors;

namespace {
constexpr double kPi  = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr float  kFill = 5.0f;

bool is_nan(double v) { return std::isnan(v); }
}  // namespace

// 基础: 半度栅格 (避开缝点) → 三扇区最小距离与计数
TEST(LidarSectors, BasicFrontThreeSectorsMinAndCounts) {
    // -180.5° 起, 1° 步进, 360 点 (半度落点, 无名义边界值)
    std::vector<float> scan(360, kFill);
    scan[150] = 2.0f;  // θ=-30.5° (左扇区)
    scan[181] = 1.5f;  // θ=+0.5°  (中扇区)
    scan[211] = 2.5f;  // θ=+30.5° (右扇区)
    const auto r = compute_lidar_sectors(scan, -180.5 * kDeg, kDeg, 0.15, 12.0, LidarSectorConfig{});
    EXPECT_EQ(r.n_input, 360);
    // ±45° 内半度点: L/C/R 各 30 个 (其余 270 个在扇区外, 不计数)
    EXPECT_EQ(r.left.n_valid, 30);
    EXPECT_EQ(r.center.n_valid, 30);
    EXPECT_EQ(r.right.n_valid, 30);
    EXPECT_DOUBLE_EQ(r.left.min_m, 2.0);
    EXPECT_DOUBLE_EQ(r.center.min_m, 1.5);
    EXPECT_DOUBLE_EQ(r.right.min_m, 2.5);
}

// 缝点语义: 左闭右开 + 最右端闭合 (±0.001° 偏移构造)
TEST(LidarSectors, SeamPointsFollowHalfOpenRule) {
    auto one = [](double d, float r) {
        return compute_lidar_sectors({r}, d * kDeg, kDeg, 0.15, 12.0, LidarSectorConfig{});
    };
    // 左扇区: [-45,-15) —— 内侧 -44.999° 计, 左端外 -45.001° 不计
    const auto a = one(-44.999, 2.0f);
    EXPECT_EQ(a.left.n_valid, 1);
    EXPECT_DOUBLE_EQ(a.left.min_m, 2.0);
    // 左/中分界 -15°: 内侧 -15.001° → L; 外侧 -14.999° → C
    const auto b = one(-15.001, 3.0f);
    EXPECT_EQ(b.left.n_valid, 1);
    EXPECT_DOUBLE_EQ(b.left.min_m, 3.0);
    const auto c = one(-14.999, 4.0f);
    EXPECT_EQ(c.center.n_valid, 1);
    EXPECT_DOUBLE_EQ(c.center.min_m, 4.0);
    // 中/右分界 15°: 内侧 14.999° → C; 外侧 15.001° → R
    const auto d = one(14.999, 5.0f);
    EXPECT_EQ(d.center.n_valid, 1);
    EXPECT_DOUBLE_EQ(d.center.min_m, 5.0);
    const auto e = one(15.001, 6.0f);
    EXPECT_EQ(e.right.n_valid, 1);
    EXPECT_DOUBLE_EQ(e.right.min_m, 6.0);
    // 右扇区 [15,45]: 右端内侧 44.999° 计; 右端外 45.001° 不计
    const auto f = one(44.999, 7.0f);
    EXPECT_EQ(f.right.n_valid, 1);
    EXPECT_DOUBLE_EQ(f.right.min_m, 7.0);
    const auto g = one(-45.001, 1.0f);
    EXPECT_EQ(g.left.n_valid, 0);
    EXPECT_TRUE(is_nan(g.left.min_m));
    const auto h = one(45.001, 8.0f);
    EXPECT_EQ(h.right.n_valid, 0);
    EXPECT_TRUE(is_nan(h.right.min_m));
}

// 反向扫描 (angle_increment < 0) 同样处理
TEST(LidarSectors, NegativeIncrementScanHandled) {
    // +45.5° 起, -1° 步进, 91 点 (覆盖 +45.5° ~ -44.5°)
    std::vector<float> scan(91, kFill);
    scan[15] = 2.5f;  // θ=+30.5° (R)
    scan[45] = 1.5f;  // θ=+0.5°  (C)
    scan[76] = 2.0f;  // θ=-30.5° (L)
    const auto r = compute_lidar_sectors(scan, 45.5 * kDeg, -kDeg, 0.15, 12.0, LidarSectorConfig{});
    EXPECT_EQ(r.left.n_valid, 30);
    EXPECT_EQ(r.center.n_valid, 30);
    EXPECT_EQ(r.right.n_valid, 30);
    EXPECT_DOUBLE_EQ(r.left.min_m, 2.0);
    EXPECT_DOUBLE_EQ(r.center.min_m, 1.5);
    EXPECT_DOUBLE_EQ(r.right.min_m, 2.5);
}

// 无效点过滤: NaN / inf / 超量程 / 0.0(无回波标记) 一律不算
TEST(LidarSectors, InvalidPointsAreFilteredOut) {
    const float nanf = std::numeric_limits<float>::quiet_NaN();
    std::vector<float> scan(360, nanf);
    scan[180] = 0.75f;                                  // θ=0°, 唯一有效点
    scan[182] = std::numeric_limits<float>::infinity(); // inf
    scan[183] = 20.0f;                                  // > range_max
    scan[184] = 0.10f;                                  // < range_min (0.15)
    const auto r = compute_lidar_sectors(scan, -180.0 * kDeg, kDeg, 0.15, 12.0, LidarSectorConfig{});
    EXPECT_EQ(r.center.n_valid, 1);
    EXPECT_DOUBLE_EQ(r.center.min_m, 0.75);
    EXPECT_EQ(r.left.n_valid, 0);
    EXPECT_EQ(r.right.n_valid, 0);

    // 0.0 是部分驱动的"无回波"标记; range_min=0 的配置下也必须跳过
    const auto r0 = compute_lidar_sectors({0.0f, 1.0f, 2.0f}, 0.0, kDeg, 0.0, 12.0, LidarSectorConfig{});
    EXPECT_EQ(r0.center.n_valid, 2);
    EXPECT_DOUBLE_EQ(r0.center.min_m, 1.0);
}

// fail-closed 总闸: 全无效 ⇒ 三扇区全部 NaN (绝不给 0 / "畅通"读数)
TEST(LidarSectors, AllInvalidYieldsNaN) {
    std::vector<float> scan(100, std::numeric_limits<float>::quiet_NaN());
    const auto r = compute_lidar_sectors(scan, -kPi, kDeg, 0.15, 12.0, LidarSectorConfig{});
    EXPECT_EQ(r.n_input, 100);
    EXPECT_TRUE(is_nan(r.left.min_m));
    EXPECT_TRUE(is_nan(r.center.min_m));
    EXPECT_TRUE(is_nan(r.right.min_m));
    EXPECT_EQ(r.left.n_valid, 0);
    EXPECT_EQ(r.center.n_valid, 0);
    EXPECT_EQ(r.right.n_valid, 0);
}

// 空帧同样 fail-closed
TEST(LidarSectors, EmptyScanYieldsNaN) {
    const auto r = compute_lidar_sectors({}, -kPi, kDeg, 0.15, 12.0, LidarSectorConfig{});
    EXPECT_EQ(r.n_input, 0);
    EXPECT_TRUE(is_nan(r.left.min_m));
    EXPECT_TRUE(is_nan(r.center.min_m));
    EXPECT_TRUE(is_nan(r.right.min_m));
}

// 安装角外参 (偏航): 同一点因 offset 不同落入不同扇区
TEST(LidarSectors, YawOffsetMovesPointBetweenSectors) {
    LidarSectorConfig cfg{};
    auto at = [&](double off) {
        cfg.yaw_offset_deg = off;
        return compute_lidar_sectors({2.0f}, 0.0, kDeg, 0.15, 12.0, cfg);
    };
    const auto c = at(0.0);   // 0° → C
    EXPECT_EQ(c.center.n_valid, 1);
    EXPECT_DOUBLE_EQ(c.center.min_m, 2.0);
    const auto rr = at(20.0); // +20° → R
    EXPECT_EQ(rr.right.n_valid, 1);
    EXPECT_DOUBLE_EQ(rr.right.min_m, 2.0);
    EXPECT_TRUE(is_nan(rr.center.min_m));
    const auto ll = at(-20.0); // -20° → L
    EXPECT_EQ(ll.left.n_valid, 1);
    EXPECT_DOUBLE_EQ(ll.left.min_m, 2.0);
    EXPECT_TRUE(is_nan(ll.center.min_m));
}

// 角度绕 ±180° 归一化: 正确侧入扇区 / 越界侧正确剔除
TEST(LidarSectors, AngleWrapAroundHandled) {
    LidarSectorConfig cfg{};
    // -175° + 210° = +35° → 落入右扇区
    cfg.yaw_offset_deg = 210.0;
    const auto a = compute_lidar_sectors({3.0f}, -175.0 * kDeg, kDeg, 0.15, 12.0, cfg);
    EXPECT_EQ(a.right.n_valid, 1);
    EXPECT_DOUBLE_EQ(a.right.min_m, 3.0);
    // 170° + 20° = 190° → -170° (扇区外, 不得因溢出判错)
    cfg.yaw_offset_deg = 20.0;
    const auto b = compute_lidar_sectors({3.0f}, 170.0 * kDeg, kDeg, 0.15, 12.0, cfg);
    EXPECT_EQ(b.left.n_valid + b.center.n_valid + b.right.n_valid, 0);
    // -170° - 20° = -190° → +170° (扇区外)
    cfg.yaw_offset_deg = -20.0;
    const auto c = compute_lidar_sectors({3.0f}, -170.0 * kDeg, kDeg, 0.15, 12.0, cfg);
    EXPECT_EQ(c.left.n_valid + c.center.n_valid + c.right.n_valid, 0);
    // θ=180° 恰值: 合法角度但不属前向扇区
    const auto d = compute_lidar_sectors({3.0f}, 180.0 * kDeg, kDeg, 0.15, 12.0, LidarSectorConfig{});
    EXPECT_TRUE(is_nan(d.center.min_m));
}

// 同扇区多点取最小 (不是首个/末个)
TEST(LidarSectors, MinPicksSmallestWithinSector) {
    const auto r = compute_lidar_sectors({3.0f, 0.5f, 1.2f}, -10.0 * kDeg, 10.0 * kDeg, 0.15, 12.0, LidarSectorConfig{});
    EXPECT_EQ(r.center.n_valid, 3);
    EXPECT_DOUBLE_EQ(r.center.min_m, 0.5);
    EXPECT_EQ(r.left.n_valid + r.right.n_valid, 0);
}

// 配置非法 (分界倒挂 / hi<rc) ⇒ 整体 fail-closed 全 NaN
TEST(LidarSectors, InvalidConfigFailsClosed) {
    std::vector<float> scan(10, 1.0f);
    LidarSectorConfig bad{};
    bad.lc_deg = 15.0;
    bad.rc_deg = -15.0;  // 分界倒挂
    const auto r = compute_lidar_sectors(scan, -kPi, kDeg, 0.15, 12.0, bad);
    EXPECT_EQ(r.n_input, 10);
    EXPECT_TRUE(is_nan(r.left.min_m));
    EXPECT_TRUE(is_nan(r.center.min_m));
    EXPECT_TRUE(is_nan(r.right.min_m));
    EXPECT_EQ(r.left.n_valid + r.center.n_valid + r.right.n_valid, 0);

    LidarSectorConfig bad2{};
    bad2.hi_deg = 10.0;  // hi < rc
    const auto r2 = compute_lidar_sectors(scan, -kPi, kDeg, 0.15, 12.0, bad2);
    EXPECT_TRUE(is_nan(r2.center.min_m));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
