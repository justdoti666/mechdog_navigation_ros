/**
 * ultra_publish_mode — 发布模式判定 seam (U-1, 2026-10-05 复核修复)
 *
 * 背景: use_gpio=true 但 GPIO 不可用时 (权限/接线/机型/未编译), 旧实现
 *   (`ultrasonic_node.cpp`: `if (!real) simulate(&msg);`) 把"真读失败"与
 *   "显式模拟"混成一条路 ⇒ 模拟随机数带 valid=true 进安全链, 与文档承诺的
 *   fail-closed 相反 (U-1 复核报告, BLOCKER/安全方向)。本 seam 把判定抽成
 *   纯函数, 单测锁定三态语义。
 *
 * 三态:
 *   RealGpio   —— 真读 (use_gpio && 编译了 USE_GPIO && gpiochip 已打开)
 *   AllInvalid —— 全通道发无效读数 (要求真读但拿不到 ⇒ fail-closed, 绝不模拟)
 *   Simulated  —— 显式模拟 (use_gpio=false; PC/WSL 链路默认行为, 不变)
 *
 * ⚠ 先红桩 (本笔): 恒 AllInvalid (最保守 —— 绝不放行可能为假的数据);
 *   功能用例 T1/T2/T6 预期红, fail-closed 用例 T3/T4/T5 绿。实现见下一笔。
 */
#pragma once

namespace mechdog_ultra {

enum class PublishMode { RealGpio, AllInvalid, Simulated };

// use_gpio      : ROS 参数 (用户是否要求真读)
// gpio_compiled : 本二进制是否编入 USE_GPIO
// chip_open     : gpiochip 是否成功打开 (未编译时传 false)
inline PublishMode resolve_publish_mode(bool use_gpio, bool gpio_compiled, bool chip_open) {
    (void)use_gpio;
    (void)gpio_compiled;
    (void)chip_open;
    return PublishMode::AllInvalid;   // 先红桩: 恒最保守
}

}  // namespace mechdog_ultra
