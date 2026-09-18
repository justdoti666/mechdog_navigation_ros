#!/usr/bin/env bash
set +u
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash 2>/dev/null
source /home/chj/astra_ws/install/setup.bash 2>/dev/null
source /home/chj/mdw_ws/install/setup.bash 2>/dev/null
pkill -9 -f astra_camera_node; sleep 4
echo "-> 关彩色 / 开红外 重起"
setsid ros2 launch astra_camera astra.launch.xml enable_color:=false enable_ir:=true > /home/chj/astra_launch.log 2>&1 < /dev/null &
sleep 32
echo "相机节点数=$(pgrep -c -f '[a]stra_camera_node' || echo 0)"
echo "--- 红外 (12s) ---"; timeout 14 ros2 topic hz /camera/ir/image_raw 2>&1 | tail -2
echo "--- 深度 (12s) ---"; timeout 14 ros2 topic hz /camera/depth/image_raw 2>&1 | tail -2
echo "--- 日志相关行 ---"; grep -aiE "ir |infrared|color|started" /home/chj/astra_launch.log | tail -5
