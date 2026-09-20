#!/usr/bin/env bash
set +u
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash 2>/dev/null
source /home/chj/astra_ws/install/setup.bash 2>/dev/null
source /home/chj/mdw_ws/install/setup.bash 2>/dev/null
echo "== 启动前: camera=$(pgrep -c -f '[a]stra_camera_node' || echo 0) safety=$(pgrep -c -f '[s]afety_node' || echo 0) view=$(pgrep -c -f report_view.py || echo 0)"
# 相机: 无数据才重起
if ! timeout 6 ros2 topic hz /camera/depth/image_raw 2>&1 | grep -q "average rate"; then
  echo "-> 相机无数据, 重起(红外模式)"
  pkill -9 -f astra_camera_node; sleep 4
  setsid ros2 launch astra_camera astra.launch.xml enable_color:=false enable_ir:=true > /home/chj/astra_launch.log 2>&1 < /dev/null &
  sleep 34
fi
# 节点
if [ "$(pgrep -c -f '[s]afety_node' || echo 0)" = "0" ]; then
  echo "-> 节点未跑, 启动"
  setsid ros2 run mechdog_navigation_ros safety_node --ros-args \
    -p use_simulated:=false -p depth_source:=topic -p enable_pointcloud:=true \
    -p camera_height_m:=0.90 -p cloud_z:=0.0 -p ground_prior_window:=0.60 \
    -p cloud_pitch_rad:=0.2618 -p cloud_roll_rad:=0.0 -p ground_fit_method:=cell \
    > /home/chj/report_node.log 2>&1 < /dev/null &
  sleep 15
fi
# 窗口
if [ "$(pgrep -c -f report_view.py || echo 0)" = "0" ]; then
  echo "-> 窗口未跑, 启动"
  setsid python3 /home/chj/pi_report_view.py > /home/chj/report_view.log 2>&1 < /dev/null &
  sleep 9
fi
echo "== 启动后: camera=$(pgrep -c -f '[a]stra_camera_node' || echo 0) safety=$(pgrep -c -f '[s]afety_node' || echo 0) view=$(pgrep -c -f report_view.py || echo 0)"
echo "-- 话题 --"; for t in /camera/depth/image_raw /camera/ir/image_raw /safety/terrain_map; do
  printf "%-30s " "$t"; timeout 8 ros2 topic hz $t 2>&1 | head -1
done
echo "-- 节点感知(含楔形统计) --"; grep -a "感知:" /home/chj/report_node.log | tail -1 | cut -c1-260
