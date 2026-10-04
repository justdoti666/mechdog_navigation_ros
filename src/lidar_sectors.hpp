/**
 * S2 (2026-10-04): 雷达扇区消费 —— 离线首版 (纯计算, 无 ROS 依赖)。
 *
 * 背景: /scan (RPLIDAR A1M8) 已在 safety_node 订阅但零消费 (只存 scan_ranges_ + 日志计数)。
 * 口径依据: 《可行性_深度退化时超声与雷达兜底_2026-09-25》 §4-S2:
 *   用 scan 算前向 L/C/R 扇区最小距离 (+安装角外参 ⇒ base_link), 作第三路"取近";
 *   建议仅退化期启用。接入节点/融合 = 行为变化 ⇒ 师兄会签后再动。
 * 本文件只做"帧 → 扇区最小距离"的纯函数, 配套单测锁定语义; 不接入任何节点。
 *
 * 语义 (安全相关, 逐条由 test/test_lidar_sectors.cpp 锁定):
 *   ① 扇区 (度, base_link 系): L=[lo, lc) · C=[lc, rc) · R=[rc, hi]
 *      (左闭右开 + 最右端闭合; 每个点只归一个扇区)
 *      默认 ±45°/±15° —— 对齐超声三颗布局 (中心 -30/0/+30 各 ±15° 宽), 现场标定后可调
 *   ② 外参: v1 仅偏航 yaw_offset_deg (雷达系角度 + 偏航 = base_link 系角度; 默认 0, 待现场标定)
 *   ③ 无效点: NaN/±inf/0.0(部分驱动的"无回波"标记)/< range_min/> range_max 一律跳过
 *   ④ fail-closed: 扇区无有效点 ⇒ min_m = NaN (绝不给 0/"畅通"读数); 配置非法 ⇒ 整体全 NaN
 *   ⑤ 角度绕 ±180° 归一化; angle_increment 为负 (反向扫描) 同样成立
 * 单位: 输入 m (LaserScan 口径), 输出 m。
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace mechdog_ros {

// 度数归一化到 (-180, 180]
inline double wrap180_deg(double a_deg) {
    a_deg = std::fmod(a_deg, 360.0);
    if (a_deg <= -180.0) a_deg += 360.0;
    if (a_deg > 180.0)   a_deg -= 360.0;
    return a_deg;
}

struct LidarSectorConfig {
    double lo_deg = -45.0;        // 左扇区左界
    double lc_deg = -15.0;        // 左/中分界
    double rc_deg = 15.0;         // 中/右分界
    double hi_deg = 45.0;         // 右扇区右界
    double yaw_offset_deg = 0.0;  // 安装角外参 (雷达 → base_link; 逆时针为正; 待现场标定)
};

struct LidarSectorResult {
    double min_m = std::numeric_limits<double>::quiet_NaN();  // 无有效点 = NaN (fail-closed)
    int    n_valid = 0;
};

struct LidarSectors {
    LidarSectorResult left, center, right;
    int n_input = 0;  // 输入点数 (含无效)
};

// scan 帧 → 前向 L/C/R 扇区最小距离 (m)。
inline LidarSectors compute_lidar_sectors(const std::vector<float>& ranges,
                                          double angle_min, double angle_increment,
                                          double range_min, double range_max,
                                          const LidarSectorConfig& cfg) {
    LidarSectors out;
    out.n_input = static_cast<int>(ranges.size());

    // 配置非法 ⇒ 全 NaN (fail-closed; 宁可报"无数据", 绝不报错扇区)
    if (!(cfg.lo_deg <= cfg.lc_deg && cfg.lc_deg <= cfg.rc_deg && cfg.rc_deg <= cfg.hi_deg)) {
        return out;
    }

    constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        const double r = static_cast<double>(ranges[i]);
        if (!std::isfinite(r) || r <= 0.0)  continue;  // NaN/±inf/0.0(无回波标记)
        if (r < range_min || r > range_max) continue;  // 超出驱动声明的量程
        const double a_deg = wrap180_deg(
            (angle_min + static_cast<double>(i) * angle_increment) * kRad2Deg + cfg.yaw_offset_deg);
        LidarSectorResult* s = nullptr;
        if      (a_deg >= cfg.lo_deg && a_deg <  cfg.lc_deg) s = &out.left;
        else if (a_deg >= cfg.lc_deg && a_deg <  cfg.rc_deg) s = &out.center;
        else if (a_deg >= cfg.rc_deg && a_deg <= cfg.hi_deg) s = &out.right;  // 最右端闭合
        if (s == nullptr) continue;  // 前向扇区之外 (含背后/侧面) —— 本版不计
        s->n_valid += 1;
        if (std::isnan(s->min_m) || r < s->min_m) s->min_m = r;
    }
    return out;
}

} // namespace mechdog_ros
