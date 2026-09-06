/**
 * R4 (REVIEW): 安全层启动预热 —— 等超声波底部线程产出首帧再进入融合主循环。
 *
 * 背景 (见 ../docs/REVIEW_FIXPLAN.md R4): 启动时 UltrasonicArrayDriver 的
 * is_fall_risk() 因 bottom_have_==false 即刻返 true, 首帧 fuse() 必判悬崖 -> STOP ~150ms。
 *
 * 本函数等待底部线程就绪, 让首帧融合在传感器数据到位之后执行。
 *   - 不屏蔽 STOP (不放松 fail-closed), 只是把首帧融合时序前置 —— 属"等待数据"而非"掩盖轨迹";
 *   - 有界超时 (默认 250ms): 就绪后立即返回 (sim 下 ~50-150ms), 不额外阻塞启动;
 *   - 返回 true=就绪; false=超时仍未就绪 (调用方仍继续, 后续 is_fall_risk() 继续 fail-closed 兜底)。
 */
#pragma once

#include <chrono>
#include <thread>

#include "sensor_ultrasonic.h"

namespace mechdog_ros {

inline bool wait_for_bottom_ready(
    mechdog::UltrasonicArrayDriver& u,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(250)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (u.is_bottom_ready()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return u.is_bottom_ready();
}

} // namespace mechdog_ros
