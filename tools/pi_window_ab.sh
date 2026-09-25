#!/usr/bin/env bash
# pi_window_ab.sh —— #9 真机验收: 量「汇报窗口」与「节点」自身的 CPU
#
# 四个相位(只重启 节点/窗口, 不碰相机):
#   ① 窗口A = 旧版窗口(直取 /camera/depth/image_raw 614KB)   ② 窗口B = 新版窗口(取 /safety/depth_small 154KB)
#   ③ 节点: publish_depth_small:=false                       ④ 节点: publish_depth_small:=true
#
# 前置(部署后只用做一次):
#   bash tools/deploy_all.sh                                   # 把本目录脚本推到 Pi:/home/chj/
#   git -C ~/mdw_ws/src/mechdog_navigation_ros log --oneline -1 # 确认 Pi 上已是 v2.9.15 (含 depth_small)
#   git -C ~/mdw_ws/src/mechdog_navigation_ros show HEAD~1:tools/pi_report_view.py > /tmp/view_old.py
# 用法:
#   bash /home/chj/pi_window_ab.sh                          # 默认 旧=/tmp/view_old.py 新=/home/chj/pi_report_view.py
set +u
OLD=${1:-/tmp/view_old.py}
NEW=${2:-/home/chj/pi_report_view.py}
[ -f "$OLD" ] || { echo "缺旧窗口文件: $OLD (见脚本头部注释导出方法)"; exit 1; }
[ -f "$NEW" ] || { echo "缺新窗口文件: $NEW"; exit 1; }
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash 2>/dev/null
source /home/chj/mdw_ws/install/setup.bash 2>/dev/null

CPUT() { awk '{print $14+$15}' /proc/$1/stat 2>/dev/null || echo "-1"; }
PCT()  { awk -v d=$1 -v s=$2 'BEGIN{ if (d<0) printf "测不到(进程没了)"; else printf "%.2f%% 核 (%.0f ticks / %.0fs)", d/100.0/s*100.0, d, s }'; }

echo "== 前置检查 =="
echo -n "  深度流:   "; timeout 6 ros2 topic hz /camera/depth/image_raw 2>&1 | grep -a average | head -1
echo -n "  小图话题: "; timeout 6 ros2 topic hz /safety/depth_small   2>&1 | grep -a average | head -1

view_ab () {   # $1=窗口文件 $2=标签
  pkill -f 'pi_report_view' 2>/dev/null; sleep 2
  python3 "$1" > /tmp/ab_view_$2.log 2>&1 &
  local V=$!
  sleep 6
  local c0; c0=$(CPUT $V)
  sleep 20
  local c1; c1=$(CPUT $V)
  kill -TERM $V 2>/dev/null; sleep 1; kill -9 $V 2>/dev/null
  echo "窗口[$2] $(PCT $((c1-c0)) 20)   (日志 /tmp/ab_view_$2.log)"
}

start_node () {  # $1 = publish_depth_small 取值; 返回节点 pid
  pkill -9 -f '[s]afety_node' 2>/dev/null; sleep 2
  setsid ros2 run mechdog_navigation_ros safety_node --ros-args \
    -p use_simulated:=false -p depth_source:=topic -p camera_height_m:=0.76 \
    -p cloud_z:=0.0 -p cloud_pitch_rad:=0.0 -p cloud_roll_rad:=0.0 \
    -p ground_fit_method:=cell -p publish_depth_small:=$1 \
    > /home/chj/report_node.log 2>&1 < /dev/null &
  sleep 12
  pgrep -f 'lib/mechdog_navigation_ros/safety_node' | head -1
}

echo "== 相位① 窗口A: 旧版(614KB 原图) =="     ; view_ab "$OLD" old
echo "== 相位② 窗口B: 新版(小图) =="           ; view_ab "$NEW" new
echo "== 相位③ 节点: 关小图 =="
NP=$(start_node false); c0=$(CPUT $NP); sleep 20; c1=$(CPUT $NP)
echo "节点[small=false] $(PCT $((c1-c0)) 20)   (pid=$NP)"
echo "== 相位④ 节点: 开小图 =="
NP=$(start_node true);  c0=$(CPUT $NP); sleep 20; c1=$(CPUT $NP)
echo "节点[small=true ] $(PCT $((c1-c0)) 20)   (pid=$NP)"

echo "== 恢复现场(节点 + 窗口) =="
bash /home/chj/pi_start_node.sh > /dev/null 2>&1 &
sleep 4
bash /home/chj/pi_view_up.sh > /dev/null 2>&1 &
sleep 2
echo "做完把上面 4 行数字发我。"
