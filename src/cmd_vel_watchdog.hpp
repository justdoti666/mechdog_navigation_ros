/**
 * v2.9.23 (批B B12 残, 二轮审查): 底盘桥"上游静默"判据 —— 纯函数, 便于单测。
 *
 * 背景: /cmd_vel 静默 (上游崩溃/闸门挂掉) 时, STM32 会**永久保持最后一条指令**;
 *   桥层此前只在"发送失败"时兜底 (T-B4/v2.9.21), 对"上游不发"无检测。
 *   修复: chassis_bridge_node 以 100ms 周期检查 last_cmd 时间戳, 静默超
 *   cmd_vel_timeout_ms (默认 500) ⇒ 告警一次 + 周期性零速兜底帧 (幂等)。
 *
 * 判据语义 (边界由单测锁定):
 *   - 严格大于 timeout 才算静默 (恰好等于不算);
 *   - last_ns <= 0 (未设起点) 一律不判 —— 计时起点由调用方负责 (节点用构造时刻)。
 */
#pragma once

#include <cstdint>

namespace mechdog_ros {

/** true = 上游静默 (距最后一条指令 > timeout_ns) */
inline bool upstream_silent(int64_t last_ns, int64_t now_ns, int64_t timeout_ns) {
    return last_ns > 0 && (now_ns - last_ns) > timeout_ns;
}

} // namespace mechdog_ros
