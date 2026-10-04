/**
 * S2 (2026-10-04): 雷达扇区消费 —— 离线首版 (纯计算, 无 ROS 依赖)。
 *
 * !!! 先红桩: 本提交只放接口与类型, 不实现逻辑 (全 NaN 返回)。
 *     功能用例在桩上应全部失败 (先红), 实现见同批 green 提交。
 *
 * 语义与设计见 docs/LIDAR_SECTORS.md (green 提交补齐)。
 */
#pragma once

#include <cstddef>
#include <limits>
#include <vector>

namespace mechdog_ros {

struct LidarSectorConfig {
    double lo_deg = -45.0;        // 左扇区左界
    double lc_deg = -15.0;        // 左/中分界
    double rc_deg = 15.0;         // 中/右分界
    double hi_deg = 45.0;         // 右扇区右界
    double yaw_offset_deg = 0.0;  // 安装角外参 (雷达 → base_link; 待现场标定)
};

struct LidarSectorResult {
    double min_m = std::numeric_limits<double>::quiet_NaN();  // 无有效点 = NaN (fail-closed)
    int    n_valid = 0;
};

struct LidarSectors {
    LidarSectorResult left, center, right;
    int n_input = 0;  // 输入点数 (含无效)
};

// scan 帧 → 前向 L/C/R 扇区最小距离 (m)。桩: 仅回 n_input, 其余全 NaN/0。
inline LidarSectors compute_lidar_sectors(const std::vector<float>& ranges,
                                          double /*angle_min*/, double /*angle_increment*/,
                                          double /*range_min*/, double /*range_max*/,
                                          const LidarSectorConfig& /*cfg*/) {
    LidarSectors out;
    out.n_input = static_cast<int>(ranges.size());
    return out;
}

} // namespace mechdog_ros
