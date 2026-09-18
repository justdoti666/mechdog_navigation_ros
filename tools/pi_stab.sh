#!/usr/bin/env bash
set +u
export ROS_DOMAIN_ID=42
echo "=== 连接确认 ==="; echo "host=$(hostname)  ip=$(hostname -I | cut -d' ' -f1)"
echo "=== 服务状态 ==="
echo "camera=$(pgrep -c -f '[a]stra_camera_node' || echo 0)  safety=$(pgrep -c -f '[s]afety_node' || echo 0)  view=$(pgrep -c -f report_view.py || echo 0)"
echo "=== WiFi 省电 (掉线常见元凶) ==="
iw dev wlan0 get power_save 2>/dev/null || iwconfig wlan0 2>/dev/null | grep -i power
echo "--- 尝试关闭(免密 sudo 才行) ---"
sudo -n iw dev wlan0 set power_save off 2>&1 | tail -1
echo "关闭后: $(iw dev wlan0 get power_save 2>/dev/null || echo '(读不到)')"
echo "=== WiFi 质量/速率 ==="
iwconfig wlan0 2>/dev/null | grep -iE "quality|bit rate|signal" | head -3
echo "=== 最近内核无线报错 ==="
dmesg 2>/dev/null | tail -40 | grep -iE "brcmfmac|wlan|disconnect|deauth" | tail -4 || echo "(需 sudo 读 dmesg)"
echo "=== 话题 === "
source /opt/ros/jazzy/setup.bash 2>/dev/null; source /home/chj/astra_ws/install/setup.bash 2>/dev/null
timeout 10 ros2 topic hz /camera/ir/image_raw 2>&1 | head -1
timeout 10 ros2 topic hz /camera/depth/image_raw 2>&1 | head -1
