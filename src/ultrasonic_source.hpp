/**
 * v2.5: 超声来源解析 (纯函数, 便于单测)。
 *
 * 背景 (真机实测 2026-09-13, Pi 5B): 在 Pi 上建 ROS 工作区时没有 USE_WIRINGPI →
 * UltrasonicArrayDriver 是**编译期模拟器**; 但模拟数据带 valid=true, 融合层分辨不出来,
 * 而 simulate_measure() 里有一句"底部传感器 5% 概率模拟悬崖" → 实测 245 帧里出现
 * 14 帧 cliff=YES (5.7%) 的**假 STOP**。安全链里绝不能有"看起来像真的随机数"。
 *
 * 解析规则 (请求值 → 实际来源; 任何非 "simulated" 请求都**不会**得到 simulated):
 *   "none"      → none
 *   "topic"     → topic   (此刻无发布者也保持 topic, 由上层在超时后禁用)
 *   "hardware"  → hardware 若 GPIO 就绪, 否则 none   (绝不静默回落模拟)
 *   "simulated" → simulated 仅当 (整机模拟模式 || 显式 allow_simulated)
 *   "auto"      → use_simulated      ? simulated
 *               : /ultrasonic 有发布者 ? topic
 *               : GPIO 就绪            ? hardware
 *               :                        none
 */
#pragma once

#include <cstddef>
#include <string>

namespace mechdog_ros {

inline std::string resolve_ultrasonic_source(const std::string& requested,
                                             bool use_simulated,
                                             bool hw_available,
                                             std::size_t topic_publishers,
                                             bool allow_simulated) {
    if (requested == "none")      return "none";
    if (requested == "topic")     return "topic";
    if (requested == "hardware")  return hw_available ? "hardware" : "none";
    if (requested == "simulated") {
        return (use_simulated || allow_simulated) ? "simulated" : "none";
    }

    // "auto" (含未知取值 → 按 auto 处理, 保守)
    if (use_simulated)          return "simulated";
    if (topic_publishers > 0)   return "topic";
    if (hw_available)           return "hardware";
    return "none";
}

} // namespace mechdog_ros
