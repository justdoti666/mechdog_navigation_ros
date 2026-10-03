/**
 * v2.9.23 (批B N4, 二轮审查): safety_node 运行期参数白名单 —— 纯函数, 便于单测。
 *
 * 背景: `ros2 param set` 对所有已声明参数都会"成功", 但只有少数成员在运行期被同步 ——
 *   其余 (话题名/几何/开关) 改了不生效 ⇒ 现场"看起来改上了"实际没有
 *   (README 曾教用户 param set 改 depth_info_topic, 永远无效)。
 *   修复: 回调对白名单外的参数显式拒绝 (successful=false + reason 说明), 白名单内即时生效。
 *
 * 白名单 = 运行期"真能即时生效"的参数集合, 与 safety_node 参数回调逐一对应 (单测锁定)。
 */
#pragma once

#include <string>

namespace mechdog_ros {

inline bool param_runtime_syncable(const std::string& name) {
    return name == "degraded_policy"        // 降级链开关 (T-A2b 起运行期可 set)
        || name == "depth_bad_streak_n"     // 深度降级阈值 (v2.9.23)
        || name == "depth_timeout_ms"       // 深度超时看门狗 (v2.9.23)
        || name == "ultrasonic_timeout_ms"  // 超声超时看门狗 + 驱动注入超时 (v2.9.23, 与 B4 同源)
        || name == "publish_depth_small"    // 汇报小图发布开关 (v2.9.23)
        || name == "grid_wedge_only"        // 视场楔形口径 (v2.9.23)
        || name == "cloud_downsample_step"; // 点云降采样步长 (v2.9.23)
}

} // namespace mechdog_ros
