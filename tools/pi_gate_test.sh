#!/usr/bin/env bash
# v2.9.3 深度质量守门 —— 真机注入测试 (复现实机事故: 深度全 0 帧)
set +u
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash 2>/dev/null
source /home/chj/mdw_ws/install/setup.bash 2>/dev/null

echo "=== 0) 停相机(避免与回放抢话题) + 重启节点(清日志) ==="
pkill -9 -f "[a]stra_camera_node"; pkill -9 -f "[s]afety_node"; sleep 3
: > /home/chj/report_node.log
setsid ros2 run mechdog_navigation_ros safety_node --ros-args \
  -p use_simulated:=false -p depth_source:=topic -p camera_height_m:=0.9 \
  -p ground_prior_window:=0.6 -p ground_fit_method:=cell \
  -p cloud_pitch_rad:=0.2618 -p cloud_roll_rad:=0.0 -p enable_pointcloud:=true \
  > /home/chj/report_node.log 2>&1 < /dev/null &
sleep 12
echo "=== A) 基线: 无帧时节点行为 ==="
grep -a "感知:" /home/chj/report_node.log | tail -1 | cut -c1-200
grep -ac "深度质量未就绪" /home/chj/report_node.log

echo "=== B) 造全 0 深度帧 (640x480 16UC1) ==="
python3 -c "import numpy as np; open('/home/chj/zero_depth.raw','wb').write(np.zeros((480,640),dtype=np.uint16).tobytes()); print('zero frame written 614400 bytes')"

echo "=== C) 回放器用法 ==="
grep -aE "argv|usage|def main|rate|Image_raw" /home/chj/pi_replay.py | head -6
