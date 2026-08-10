#!/bin/bash
# ============================================================
# 三库一体化联调 (WSL 侧): 机器人模型 + 联调节点 + RViz 可视化
# 配合 Windows 侧算法库 RGB 可视化窗口, 构成完整效果
# ============================================================
set -e
source /opt/ros/lyrical/setup.bash
cd ~/mechdog_ws
source install/setup.bash
export ROS_DOMAIN_ID=42
# WSLg GUI: 强制 X11 模式 (Wayland socket 缺失时 RViz 连不上 wl_display)
unset WAYLAND_DISPLAY
export DISPLAY=:0
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=1

echo "=== [1/4] 启动 robot_state_publisher (机器人模型) ==="
ros2 launch quadruped_description robot_description.launch.py > /tmp/rsp.log 2>&1 &
sleep 2

echo "=== [2/4] 启动联调节点 ==="
# 师兄闸门 (层1)
ros2 run quadruped_base cmd_vel_safety_gate_node > /tmp/gate.log 2>&1 &
sleep 1
# 师兄底盘桥 (层2, 串口模拟)
ros2 run quadruped_base wheel_board_bridge_node --ros-args -p serial_enabled:=false > /tmp/wbb.log 2>&1 &
sleep 1
# 本包底盘桥
ros2 run mechdog_navigation_ros chassis_bridge_node > /tmp/bridge.log 2>&1 &
sleep 1
# 本包安全层
ros2 run mechdog_navigation_ros safety_node > /tmp/safety.log 2>&1 &
sleep 2

echo "=== [3/4] enable_motion (放行闸门) ==="
ros2 topic pub -1 /robot/enable_motion std_msgs/msg/Bool "{data: true}" > /dev/null 2>&1
sleep 1

echo "=== [4/4] 启动 RViz (机器人模型可视化) ==="
RVIZ_CONFIG=$(ros2 pkg prefix quadruped_description)/share/quadruped_description/rviz/quadruped_robot.rviz
echo "RViz 配置: $RVIZ_CONFIG"
timeout 30 ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" > /tmp/rviz.log 2>&1 &
sleep 5

echo ""
echo "========================================"
echo " 一体化联调运行中 (30秒后自动结束)"
echo "   - 机器人模型: RViz 窗口 (应已弹出)"
echo "   - 控制链: safety→闸门→/cmd_vel→桥"
echo "   - 请在 Windows 端打开 RGB 可视化窗口"
echo "========================================"

# 期间周期性打印联调状态
for i in 1 2 3 4 5; do
  sleep 4
  VX=$(grep -oP "vx=\K[0-9.]+" /tmp/bridge.log 2>/dev/null | tail -1)
  UHZ=$(timeout 1 ros2 topic hz /unsafe/cmd_vel 2>/dev/null | grep -oP 'average rate: \K[0-9.]+' | head -1)
  echo "[状态 $i] /unsafe/cmd_vel=${UHZ:-?}Hz  bridge_vx=${VX:-0}"
done

echo ""
echo "========== 结果判定 =========="
BRIDGE_VX=$(grep -oP "vx=\K[0-9.]+" /tmp/bridge.log 2>/dev/null | tail -1)
if [ -n "$BRIDGE_VX" ] && [ "$BRIDGE_VX" != "0" ]; then
  echo "✅ PASS: 控制链全通 + 可视化已启动 (RViz 机器人模型)"
else
  echo "⚠️ 检查: bridge 未见非零速度"
fi

echo "========== 清理 =========="
jobs -p | xargs -r kill 2>/dev/null
wait 2>/dev/null
echo "✅ 一体化联调完成"
