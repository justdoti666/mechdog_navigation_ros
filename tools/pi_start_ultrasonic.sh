#!/usr/bin/env bash
# pi_start_ultrasonic.sh — 真机上**显式**启动 ultrasonic_node 的包装脚本 (R-2, 2026-10-11)
#
# 为什么需要它: 节点参数 use_gpio 默认 false ⇒ 忘传就发**模拟随机数**(valid=true),
#   而消息里没有"来源=模拟"标记, safety_node 认不出对面是模拟器 (复核报告 R-2)。
#   本脚本把"真机口径"固化在一处, 避免手敲漏参数。
#
# 用法:
#   bash pi_start_ultrasonic.sh                    # 4 颗全接 (默认 active=[0,1,2,3]) + GPIO 真读
#   ACTIVE=1 bash pi_start_ultrasonic.sh           # 台架只接正前一颗 (active=[1])
#   LOG=$HOME/ultra.log bash pi_start_ultrasonic.sh
#
# 说明:
#   · 需要已 `source /opt/ros/jazzy/setup.bash` + 工作区 install (与 tools/ 其他脚本同前提);
#   · 只在 **Pi GPIO 路线**(备选路线) 用; 主路线是 STM32 桥发布 /ultrasonic, 那时**不要**起本节点
#     (两个发布者会抢话题, 见 docs/STM32_ULTRASONIC_SPEC.md §4.3);
#   · 起完可用 `tools/ultra_failclosed_check.sh`(判 fail-closed) 与 `tools/ultra_capture.sh`(取证) 复核。
set -o pipefail
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
LOG="${LOG:-$HOME/ultrasonic_node.log}"
ACTIVE="${ACTIVE:-0,1,2,3}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[错误] 找不到 ros2 —— 请先 source /opt/ros/jazzy/setup.bash 与工作区 install/setup.bash" >&2
  exit 2
fi

echo "[pi_start_ultrasonic] 启动 ultrasonic_node: use_gpio:=true active:=[$ACTIVE] 日志=$LOG"
setsid ros2 run mechdog_ultrasonic ultrasonic_node --ros-args \
  -p use_gpio:=true -p "active:=[$ACTIVE]" > "$LOG" 2>&1 < /dev/null &
PID=$!
sleep 8

echo "--- 进程 ---"
pgrep -af ultrasonic_node | head -3
echo "--- 日志关键行（真读启用 / fail-closed / 模拟告警） ---"
grep -aE "真读启用|GPIO 初始化失败|模拟随机数|全通道" "$LOG" | tail -3
echo "--- /ultrasonic 一帧（可核到 nan 与 valid） ---"
timeout 10 python3 "$(dirname "${BASH_SOURCE[0]}")/ultra_frame_capture.py" 2>&1 | head -10
echo "[pi_start_ultrasonic] 已启动 (pid=$PID)。停止: kill $PID ; 取证请用 tools/ultra_capture.sh"
