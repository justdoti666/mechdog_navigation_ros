#!/usr/bin/env bash
# 下压角扫描: 起节点逐步填 cloud_pitch_rad, 看 平面valid / tilt / h0 / in_fov 何时转正
set +u
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash 2>/dev/null
source /home/chj/mdw_ws/install/setup.bash 2>/dev/null
H=0.76
for P in 0.0000 0.2618 0.4363 0.6109; do
  pkill -9 -f '[s]afety_node'; sleep 2; : > /home/chj/report_node.log
  setsid nohup ros2 run mechdog_navigation_ros safety_node --ros-args \
    -p use_simulated:=false -p depth_source:=topic \
    -p camera_height_m:=$H -p cloud_z:=0.0 \
    -p cloud_pitch_rad:=$P -p cloud_roll_rad:=0.0 \
    -p ground_fit_method:=cell > /home/chj/report_node.log 2>&1 < /dev/null &
  sleep 13
  D=$(echo "$P" | awk '{printf "%.1f", $1*57.2958}')
  PL=$(grep -a "感知:" /home/chj/report_node.log | tail -1 | sed 's/.*点云=//' | cut -c1-95)
  ST=$(timeout 6 ros2 topic echo --once /safety/status_text 2>/dev/null | head -1 | cut -c1-120)
  echo "pitch=${P}rad (${D}°) ⇒ 平面: $PL"
  echo "    status: $ST"
done
