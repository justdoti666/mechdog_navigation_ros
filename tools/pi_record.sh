#!/usr/bin/env bash
set +u
export ROS_DOMAIN_ID=42
REC=${REC:-20}
export RECORD_SECONDS=$REC
source /opt/ros/jazzy/setup.bash 2>/dev/null
source /home/chj/astra_ws/install/setup.bash 2>/dev/null
source /home/chj/mdw_ws/install/setup.bash 2>/dev/null
echo "录制时长=${REC}s"
pkill -9 -f report_view.py; sleep 2; fuser -k 8080/tcp 2>/dev/null; sleep 2
setsid python3 /home/chj/pi_report_view.py > /home/chj/report_view.log 2>&1 < /dev/null &
sleep 3
timeout $((REC + 14)) curl -s -o /dev/null http://127.0.0.1:8080/stream.mjpg
sleep 3
grep -aE "REC_START|REC_SAVED|VIEW_READY" /home/chj/report_view.log | tail -3
echo "--- 录到的文件 ---"
ls -l /home/chj/report_*.mp4 2>/dev/null | tail -2 || echo "(没有 mp4 ✗)"
