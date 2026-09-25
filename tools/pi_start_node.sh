#!/usr/bin/env bash
# 起 safety_node(H=0.76) + 取状态/感知/深度流 读数
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
sleep 15
echo "-- 状态串 --"
timeout 8 ros2 topic echo --once /safety/status_text 2>/dev/null | head -1 | cut -c1-140 | sed 's/^/  /'
echo "-- 感知行 --"
grep -a "感知:" /home/chj/report_node.log | tail -1 | sed 's/^/  /'
echo "-- 深度守门告警次数: $(grep -ac '深度质量未就绪' /home/chj/report_node.log) --"
echo "-- 深度流 12s --"
timeout 22 python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
rclpy.init(); n = Node("chk"); st = {"n":0,"t0":None,"t1":None,"ok":0,"zero":0}
def cb(m):
    t = time.time()
    a = np.frombuffer(m.data, dtype=np.uint16)
    v = int(((a >= 600) & (a <= 8000)).sum())
    if v == 0:
        st["zero"] += 1
    elif v > a.size * 0.25:
        st["ok"] += 1
    if st["t0"] is None:
        st["t0"] = t
    st["t1"] = t; st["n"] += 1
n.create_subscription(Image, "/camera/depth/image_raw", cb, 10)
t_end = time.time() + 12
while time.time() < t_end:
    rclpy.spin_once(n, timeout_sec=0.2)
d = (st["t1"] - st["t0"]) if st["t0"] and st["t1"] and st["t1"] > st["t0"] else 0
print(f"   帧={st['n']} 速率={(st['n']-1)/d if d > 0 else 0:.1f}Hz | 合格帧={st['ok']} 全零帧={st['zero']}")
rclpy.shutdown()
PY
