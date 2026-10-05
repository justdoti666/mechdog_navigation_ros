#!/usr/bin/env bash
# ultra_failclosed_check.sh — U-1 回归工装 (2026-10-05, 先红后绿)
#
# 判据: use_gpio:=true 且「拿不到 GPIO」时, /ultrasonic 帧内不得出现任何 *_valid: true
#       (fail-closed: 真读要求拿不到 ⇒ 全通道无效; 绝不回落模拟随机数)
# 适用: WSL (无 /dev/gpiochip*) 或 Pi 上 GPIO 不可用 (权限/接线/错 chip)。
#       真机 GPIO 可用时会打印 SKIP 并退出 3 (本工装只判"拿不到"路径)。
# 帧采集: 同目录 ultra_frame_capture.py (rclpy 直捕, 不依赖 ros2 CLI/daemon ——
#        lyrical 上 `ros2 topic echo` 有 XMLRPC 兼容问题: unknown tag TopicEndpointInfo)。
# 用法: bash ultra_failclosed_check.sh [证据目录]   (需先 source ROS + 工作区 install)
set -uo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
OUT="${1:-}"
LOG=/tmp/ultra_failclosed_node.log
FRAME=/tmp/ultra_failclosed_frame.txt
: > "$LOG"; : > "$FRAME"

ros2 run mechdog_ultrasonic ultrasonic_node --ros-args -p use_gpio:=true > "$LOG" 2>&1 &
PID=$!
trap 'kill -9 "$PID" 2>/dev/null; pkill -9 -x ultrasonic_node 2>/dev/null' EXIT
sleep 6
timeout 12 python3 "$SCRIPT_DIR/ultra_frame_capture.py" > "$FRAME" 2>&1
kill -9 "$PID" 2>/dev/null; pkill -9 -x ultrasonic_node 2>/dev/null; wait "$PID" 2>/dev/null

if grep -q "真读已启用" "$LOG"; then
  echo "SKIP: 本机 GPIO 可用 (节点日志: 真读已启用) —— 本工装只判「拿不到 GPIO」的 fail-closed 路径"
  exit 3
fi
if ! grep -q "front_left_cm" "$FRAME"; then
  echo "FAIL: 未取到有效 /ultrasonic 帧"
  tail -5 "$FRAME" 2>/dev/null
  tail -8 "$LOG" 2>/dev/null
  exit 2
fi
NV=$(grep -cE '_valid: true' "$FRAME")
echo "=== /ultrasonic 帧内 valid=true 计数: $NV ==="
grep -E '_cm:|_valid:|period_sec:|seq:' "$FRAME" | head -12
echo "=== 节点日志关键行 ==="
grep -E "ERROR|WARN|fail-closed|GPIO" "$LOG" | head -8
if [ -n "$OUT" ]; then
  mkdir -p "$OUT"
  cp -f "$FRAME" "$OUT/ultrasonic_frame.txt"
  cp -f "$LOG" "$OUT/node_stdout.txt"
fi
if [ "$NV" -gt 0 ]; then
  echo "FAIL: 出现 $NV 个 valid=true (fail-open 回归 / U-1 未修)"; exit 1
fi
echo "PASS: 全部通道无效 (fail-closed)"; exit 0
