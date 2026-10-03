/**
 * v2.9.23 (批B N1, 二轮审查): 超声"启动门窗口"策略 —— 纯函数, 便于单测。
 *
 * 背景: ultrasonic_source=topic 时, 首帧到达前超声不在安全链 (B4/v2.9.19 已实现), 但
 *   **动作层没有对应策略** —— 二轮审查实测: 该窗口里深度一新鲜就照走
 *   action=FORWARD vel=(0.06→0.12 m/s) (cliff=no), 而旧注释却声称"实测=STOP"(被证伪)。
 *   无悬崖层时前进 = fail-open; 本策略把窗口内的动作层统一抬到 STOP (fail-closed)。
 *
 * 语义:
 *   ultra_gate_waiting(source, ever_rx) —— topic 源且启动以来从未收到首帧 ⇒ 窗口开启。
 *     (窗口可能无界: 发布者存在但从不发帧; 期间节点保持 STOP 并周期性告警。)
 *   gate_window_action(a, waiting)     —— 窗口内一律 STOP (含 BACKWARD/TURN: 无悬崖层时
 *     任何运动都可能朝坑; 停是唯一 fail-closed 选择)。
 */
#pragma once

#include <string>

#include "sensor_fusion.h"

namespace mechdog_ros {

/** 门窗口判定: topic 源 + 从未收到首帧 */
inline bool ultra_gate_waiting(const std::string& source, bool ever_rx) {
    return source == "topic" && !ever_rx;
}

/** 窗口内的动作层策略: 一律 STOP (fail-closed; 窗口外原样透传) */
inline mechdog::NavigationAction gate_window_action(mechdog::NavigationAction a, bool waiting) {
    return waiting ? mechdog::NavigationAction::STOP : a;
}

} // namespace mechdog_ros
