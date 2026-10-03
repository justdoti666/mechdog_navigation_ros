#!/usr/bin/env bash
set +u
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash 2>/dev/null
source $HOME/astra_ws/install/setup.bash 2>/dev/null
source $HOME/mdw_ws/install/setup.bash 2>/dev/null
echo "--- 现有 report_view 进程 ---"
pgrep -af report_view.py || echo "(无)"
pkill -9 -f report_view.py; sleep 2
fuser -k 8080/tcp 2>/dev/null || true; sleep 2
echo "--- 8080 占用 ---"
(ss -ltn 2>/dev/null || netstat -ltn 2>/dev/null) | grep 8080 || echo "(空)"
rm -f $HOME/report_view.log
setsid python3 $HOME/pi_report_view.py > $HOME/report_view.log 2>&1 < /dev/null &
sleep 10
echo "--- 启动后 ---"
echo "进程: $(pgrep -c -f report_view.py || echo 0)"
echo "日志: $(tail -2 $HOME/report_view.log | tr '\n' ' ')"
(ss -ltn 2>/dev/null || netstat -ltn 2>/dev/null) | grep 8080 && echo "端口 OK" || echo "端口 未监听 ✗"
