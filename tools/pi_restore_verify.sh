#!/usr/bin/env bash
# 恢复相机(红外模式) + 节点, 验证正常帧下全链路回归
set +u
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash 2>/dev/null
source /home/chj/astra_ws/install/setup.bash 2>/dev/null
source /home/chj/mdw_ws/install/setup.bash 2>/dev/null
pkill -9 -f astra_camera_node 2>/dev/null; sleep 3
setsid ros2 launch astra_camera astra.launch.xml enable_color:=false enable_ir:=true \
  > /home/chj/cam_ir.log 2>&1 < /dev/null &
sleep 22
echo "=== 相机话题速率 ==="
for t in /camera/depth/image_raw /camera/ir/image_raw; do
  R=$(timeout 9 ros2 topic hz $t 2>/dev/null | grep -a "average rate" | head -1)
  echo "  $t -> ${R:-无数据}"
done
echo "=== 节点回归 (正常帧) ==="
bash /home/chj/pi_tidy.sh >/dev/null 2>&1; sleep 14
grep -a "感知:" /home/chj/report_node.log | tail -1 | cut -c1-230
echo "守门告警次数(正常帧应为 0): $(grep -ac "深度质量未就绪" /home/chj/report_node.log)"
