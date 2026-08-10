#!/bin/bash
# ============================================================
# 三库完整联调脚本 (mechdog_navigation + mechdog_navigation_ros + quadruped_ws)
# 运行环境: WSL Ubuntu-26.04 + ROS2 (lyrical/jazzy)
#
# 流程:
#   1. 准备 workspace: 复制三库到 ~/mechdog_ws/src
#   2. colcon build: quadruped_msgs -> 师兄栈 8 包 -> 本包
#   3. 运行时联调: safety_node -> 师兄闸门 -> /cmd_vel -> chassis_bridge
#   4. enable_motion 后验证闸门放行 + 限幅
#
# 用法: bash integration_test.sh [--skip-build] [--ros-distro lyrical]
# ============================================================
set -e

ROS_DISTRO="lyrical"
SKIP_BUILD=0
for arg in "$@"; do
  case $arg in
    --skip-build) SKIP_BUILD=1 ;;
    --ros-distro=*) ROS_DISTRO="${arg#*=}" ;;
  esac
done

source /opt/ros/$ROS_DISTRO/setup.bash
echo "ROS distro: $ROS_DISTRO"

# ---- 1. 准备 workspace ----
WS=~/mechdog_ws
SRC_MD=/mnt/d/AndrowsData/mechdog_navigation
SRC_MR=/mnt/d/AndrowsData/mechdog_navigation_ros
SRC_QW=/mnt/d/AndrowsData/quadruped_ws

if [ ! -d "$SRC_MD" ] || [ ! -d "$SRC_MR" ] || [ ! -d "$SRC_QW" ]; then
  echo "[错误] 源码目录不存在, 请确认:"
  echo "  $SRC_MD"
  echo "  $SRC_MR"
  echo "  $SRC_QW"
  exit 1
fi

if [ $SKIP_BUILD -eq 0 ]; then
  echo "=== 准备 workspace: $WS ==="
  rm -rf $WS
  mkdir -p $WS/src
  cp -r $SRC_MD $WS/src/
  cp -r $SRC_MR $WS/src/
  cp -r $SRC_QW/src/* $WS/src/
  echo "  已复制 $(ls $WS/src | wc -l) 个包:"
  ls $WS/src

  # ---- 2. colcon build ----
  cd $WS
  echo ""
  echo "=== [1/3] 构建 quadruped_msgs ==="
  colcon build --packages-select quadruped_msgs 2>&1 | tail -3

  echo ""
  echo "=== [2/3] 构建师兄栈其余 8 包 ==="
  colcon build --packages-select quadruped_base quadruped_core quadruped_gimbal \
      quadruped_perception quadruped_sensors quadruped_voice quadruped_bringup \
      quadruped_description 2>&1 | tail -4

  echo ""
  echo "=== [3/3] 构建本包 mechdog_navigation_ros ==="
  colcon build --packages-select mechdog_navigation_ros 2>&1 | tail -4
  echo "全部构建完成"
else
  cd $WS
  echo "(跳过构建, 使用已有 $WS)"
fi

source install/setup.bash
export ROS_DOMAIN_ID=42

# ---- 3. 运行时联调 ----
echo ""
echo "=== 启动: 师兄 wheel_board_bridge_node (层2) ==="
timeout 25 ros2 run quadruped_base wheel_board_bridge_node --ros-args -p serial_enabled:=false > /tmp/wbb.log 2>&1 &
sleep 2

echo "=== 启动: 师兄 cmd_vel_safety_gate_node (层1 闸门) ==="
timeout 25 ros2 run quadruped_base cmd_vel_safety_gate_node > /tmp/gate.log 2>&1 &
sleep 2

echo "=== 启动: 本包 chassis_bridge_node (simulated) ==="
timeout 25 ros2 run mechdog_navigation_ros chassis_bridge_node > /tmp/bridge.log 2>&1 &
sleep 2

echo "=== 启动: 本包 safety_node (发 /unsafe/cmd_vel) ==="
timeout 20 ros2 run mechdog_navigation_ros safety_node > /tmp/safety.log 2>&1 &
sleep 4

echo ""
echo "========== 话题列表 =========="
ros2 topic list 2>/dev/null | sort

echo ""
echo "========== /unsafe/cmd_vel 发布频率 =========="
timeout 3 ros2 topic hz /unsafe/cmd_vel 2>/dev/null | grep -oP 'average rate: \K[0-9.]+' | head -1 || echo "无"

echo ""
echo "=== 未 enable_motion: /cmd_vel 应为 0 ==="
timeout 2 ros2 topic echo /cmd_vel geometry_msgs/msg/Twist --once 2>/dev/null | grep -E "x:|z:" | head -2 || echo "(无)"

echo ""
echo "=== 发送 enable_motion=true ==="
ros2 topic pub -1 /robot/enable_motion std_msgs/msg/Bool "{data: true}" > /dev/null 2>&1
sleep 2

echo "=== enable 后: /cmd_vel (safety 输出被闸门限幅) ==="
timeout 2 ros2 topic echo /cmd_vel geometry_msgs/msg/Twist --once 2>/dev/null | grep -E "x:|z:" | head -2 || echo "(无)"

echo ""
echo "========== 本包 bridge 输出 =========="
grep "ChassisBridge" /tmp/bridge.log 2>/dev/null | tail -3 || echo "(无)"
echo "========== safety_node 融合日志 =========="
grep -E "vel=" /tmp/safety.log 2>/dev/null | tail -2 || echo "(无)"
echo "========== 师兄闸门日志 =========="
grep -iE "started|enabled|motion" /tmp/gate.log 2>/dev/null | tail -3 || echo "(无)"

echo ""
echo "========== 清理 =========="
jobs -p | xargs -r kill 2>/dev/null
wait 2>/dev/null
echo ""
echo "✅ 联调完成 (Ctrl+C 已清理所有节点)"
