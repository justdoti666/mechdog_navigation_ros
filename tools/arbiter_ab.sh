#!/usr/bin/env bash
# L0 速度仲裁层 A/B 一键复跑 (safety_node, 模拟模式, 不需要硬件)
#
# 用法:
#   bash tools/arbiter_ab.sh            # 默认 WS=$HOME/mdw_ws
#   WS=/path/to/ws bash tools/arbiter_ab.sh
#
# 前置: 该 workspace 已 colcon build 过 mechdog_navigation_ros
# 输出: $OUT/probe_true.txt · $OUT/probe_false.txt · $OUT/probe_node_*.log
#       (默认 OUT=$HOME/arbiter_ab_$(date +%Y%m%d))
#
# 真机上把 use_simulated 改成 false、depth_source 按现场配置即可复跑同一判据。
# 注: 不能 set -u —— /opt/ros/*/setup.bash 读未定义变量会直接退出。
set +u
WS="${WS:-$HOME/mdw_ws}"
OUT="${OUT:-$HOME/arbiter_ab_$(date +%Y%m%d)}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$OUT"

source /opt/ros/"${ROS_DISTRO:-jazzy}"/setup.bash
source "$WS/install/setup.bash"

run_mode() {
    local mode="$1"
    # ros2 run 是启动器: kill $! 只杀启动器, 子进程会残留并继续发 /l0/out
    # (首版踩过: 第二个模式混进第一个节点的输出) ⇒ 按进程名清干净。
    pkill -f 'safety_node' 2>/dev/null
    sleep 1
    ros2 run mechdog_navigation_ros safety_node --ros-args \
        -p use_simulated:=true \
        -p arbiter_enable:="$mode" \
        -p cmd_vel_topic:=/l0/out \
        -p desired_cmd_topic:=/l0/desired \
        -p enable_pointcloud:=false \
        -p publish_depth_small:=false \
        -p enable_rgb:=false \
        > "$OUT/probe_node_${mode}.log" 2>&1 &
    local pid=$!
    sleep 5
    python3 "$HERE/arbiter_probe.py" "$mode" > "$OUT/probe_${mode}.txt" 2>&1
    local rc=$?
    pkill -f 'safety_node' 2>/dev/null
    wait "$pid" 2>/dev/null
    echo "===== arbiter_enable=${mode} (probe rc=${rc}) ====="
    cat "$OUT/probe_${mode}.txt"
    echo "--- 启动横幅:"
    grep -m1 "L0 速度仲裁" "$OUT/probe_node_${mode}.log" || echo "(无横幅)"
    echo
    return "$rc"
}

rc_all=0
run_mode true || rc_all=1
run_mode false || rc_all=1
echo "输出目录: $OUT"
exit "$rc_all"
