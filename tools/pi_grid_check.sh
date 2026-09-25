#!/usr/bin/env bash
set +u
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash 2>/dev/null
source /home/chj/mdw_ws/install/setup.bash 2>/dev/null
pkill -9 -f '[s]afety_node' 2>/dev/null
sleep 2
: > /home/chj/report_node.log
setsid ros2 run mechdog_navigation_ros safety_node --ros-args \
  -p use_simulated:=false -p depth_source:=topic -p camera_height_m:=0.76 \
  -p cloud_z:=0.0 -p cloud_pitch_rad:=0.0 -p cloud_roll_rad:=0.0 \
  -p ground_fit_method:=cell > /home/chj/report_node.log 2>&1 < /dev/null &
sleep 14
echo "=== (1) safety 话题 ==="
timeout 8 ros2 topic list 2>/dev/null | grep -a safety | sed 's/^/  /'
echo "=== (2) terrain_grid 单帧大小 + 类别直方图 ==="
timeout 25 python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import Image
rclpy.init(); n = Node("gridchk"); got = {}
def cb(m):
    got['sz'] = len(m.data); got['wh'] = (m.width, m.height); got['enc'] = m.encoding
    d = bytes(m.data)
    got['h'] = {k: d.count(v) for k, v in [("Unknown",0),("Trav",1),("Up",2),("Down",3),("Steep",4)]}
n.create_subscription(Image, "/safety/terrain_grid", cb, 10)
t = time.time()
while time.time()-t < 8 and 'sz' not in got:
    rclpy.spin_once(n, timeout_sec=0.2)
if 'sz' in got:
    sz = got['sz']; w, h = got['wh']
    print(f"  {w}x{h} {got['enc']} = {sz} 字节/帧  => 10Hz 时 {sz*10*8/1e6:.3f} Mbit/s")
    print("  类别: " + " ".join(f"{k}={v}" for k, v in got['h'].items()))
else:
    print("  !! 8 秒内没收到 terrain_grid")
rclpy.shutdown()
PY
echo "=== (3) 频率与延迟 (新 vs 旧) ==="
for t in /safety/terrain_grid /safety/terrain_map; do
  echo -n "  $t  频率: "; timeout 9 ros2 topic hz $t 2>&1 | grep -a "average rate" | head -1 || echo "(超时)"
  echo -n "  $t  延迟: "; timeout 9 ros2 topic delay $t 2>&1 | grep -a "average delay" | head -1 || echo "(超时)"
done
echo "=== (4) 状态串(判定未变) ==="
timeout 8 ros2 topic echo --once /safety/status_text 2>/dev/null | head -1 | cut -c1-130 | sed 's/^/  /'
