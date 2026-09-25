#!/usr/bin/env bash
# ultra_gate_check.sh — B4 超声"启动门"回归（无需硬件；PC / Pi 通用）
#
# 验证 (B4, v2.9.19):
#   ① 启动即"等待 /ultrasonic 首帧" —— 首帧前超声不接入安全链
#      (旧版启动即接入 ⇒ "有发布者但从未发帧"窗口里会吃到内部模拟随机数)
#   ② 首帧到达 ⇒ "超声重新接入安全链"
#   ③ 之后停发 >500ms ⇒ "超声退出安全链"（旧行为保留）
#   ④ silent 场景: 有发布者但 5s 无首帧 ⇒ 一次性 WARN（现场可诊断）
#
# 用法:  bash ultra_gate_check.sh [silent|once]      # 默认 once
# 前置:  source /opt/ros/<distro>/setup.bash && source <ws>/install/setup.bash
set +u
MODE="${1:-once}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

python3 - "$MODE" <<'PY' &
import sys, time
import rclpy
from rclpy.node import Node
from mechdog_ultrasonic.msg import UltrasonicArray
mode = sys.argv[1]
rclpy.init()
n = Node('b4_dummy_pub')
pub = n.create_publisher(UltrasonicArray, '/ultrasonic', 10)
if mode == 'once':
    time.sleep(3.0)
    m = UltrasonicArray()
    m.stamp = n.get_clock().now().to_msg()
    m.front_left_cm = 120.0;   m.front_left_valid = True
    m.front_center_cm = 100.0; m.front_center_valid = True
    m.front_right_cm = 120.0;  m.front_right_valid = True
    m.bottom_cm = 20.0;        m.bottom_valid = True
    m.period_sec = 0.1; m.seq = 1
    pub.publish(m)
    n.get_logger().info('sent one /ultrasonic frame')
time.sleep(11)   # 总寿命 14s: 让脚本 wait 时自然退出, 避免 kill 产生 Killed 噪声
n.destroy_node(); rclpy.shutdown()
PY
PUB=$!
sleep 2   # 给发布者留 DDS 发现时间
: > /tmp/b4_node.log
timeout 16 ros2 run mechdog_navigation_ros safety_node --ros-args \
  -p use_simulated:=false -p depth_source:=topic -p ultrasonic_source:=topic \
  -p publish_depth_small:=false > /tmp/b4_node.log 2>&1 &
sleep 13
pkill -9 -x safety_node 2>/dev/null        # -x 精确匹配进程名: 不误杀 ros2 启动器/timeout, 无 Killed 噪声
wait $PUB 2>/dev/null                       # 发布者 14s 自行退出
echo "=== B4 判据 (mode=$MODE) ==="
echo "[1] 等待首帧       : $(grep -ac '等待 /ultrasonic 首帧' /tmp/b4_node.log) 次"
grep -a '等待 /ultrasonic 首帧' /tmp/b4_node.log | head -1
echo "[2] 重新接入安全链 : $(grep -ac '重新接入安全链' /tmp/b4_node.log) 次"
grep -a '重新接入安全链' /tmp/b4_node.log | head -1
echo "[3] 退出安全链     : $(grep -ac '退出安全链' /tmp/b4_node.log) 次"
grep -a '退出安全链' /tmp/b4_node.log | head -1
echo "[4] 5s 无首帧告警  : $(grep -ac '未收到任何 /ultrasonic' /tmp/b4_node.log) 次"
grep -a '未收到任何' /tmp/b4_node.log | head -1
echo "== 期望: once => [1][2][3]>=1, [4]=0 ; silent => [1][4]>=1, [2][3]=0 =="
