#!/usr/bin/env bash
set +u
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash 2>/dev/null
source /home/chj/astra_ws/install/setup.bash 2>/dev/null
source /home/chj/mdw_ws/install/setup.bash 2>/dev/null
echo "清理前: safety_node=$(pgrep -c -f '[s]afety_node' || echo 0)"
pkill -9 -f safety_node; sleep 3
setsid ros2 run mechdog_navigation_ros safety_node --ros-args \
  -p use_simulated:=false -p depth_source:=topic -p enable_pointcloud:=true \
  -p camera_height_m:=0.750 -p cloud_z:=0.0 \
  -p cloud_pitch_rad:=0.0 -p cloud_roll_rad:=0.0 -p ground_fit_method:=cell \
  > /home/chj/report_node.log 2>&1 < /dev/null &
sleep 12
echo "清理后: safety_node=$(pgrep -c -f '[s]afety_node' || echo 0)  report_view=$(pgrep -c -f report_view.py || echo 0)  camera=$(pgrep -c -f '[a]stra_camera_node' || echo 0)"
echo "IR 速率:"; timeout 10 ros2 topic hz /camera/ir/image_raw 2>&1 | head -1
echo "terrain_map 速率:"; timeout 10 ros2 topic hz /safety/terrain_map 2>&1 | head -1
grep -a "感知:" /home/chj/report_node.log | tail -1 | cut -c1-175
