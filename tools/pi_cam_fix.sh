#!/usr/bin/env bash
set +u
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash 2>/dev/null
source /home/chj/astra_ws/install/setup.bash 2>/dev/null
source /home/chj/mdw_ws/install/setup.bash 2>/dev/null
echo "=== 现状 ==="
echo "camera=$(pgrep -c -f '[a]stra_camera_node' || echo 0)  safety=$(pgrep -c -f '[s]afety_node' || echo 0)  view=$(pgrep -c -f report_view.py || echo 0)"
echo "--- USB ---"; lsusb | grep -c 2bc5 | sed 's/^/astra 设备数: /'
echo "--- 相机日志尾 (看死因) ---"; tail -5 /home/chj/astra_launch.log
echo "=== 重新拉起相机 (关彩色/开红外) ==="
pkill -9 -f astra_camera_node; sleep 4
setsid ros2 launch astra_camera astra.launch.xml enable_color:=false enable_ir:=true > /home/chj/astra_launch.log 2>&1 < /dev/null &
sleep 34
echo "camera=$(pgrep -c -f '[a]stra_camera_node' || echo 0)"
echo "--- IR ---"; timeout 12 ros2 topic hz /camera/ir/image_raw 2>&1 | head -1
echo "--- 深度 ---"; timeout 12 ros2 topic hz /camera/depth/image_raw 2>&1 | head -1
echo "--- 节点是否恢复 ---"; grep -a "感知:" /home/chj/report_node.log | tail -1 | cut -c1-150
