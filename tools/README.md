# tools/ — 汇报用实时可视化（Pi 端）

三格实时画面：**红外（与深度同视场） / 深度（JET 伪彩） / 可行度 2.5D**，
底部状态栏显示 `trav/up/down/steep/unknown + 平面 tilt/h0`。
页面走 **MJPEG (:8080)**，浏览器或 Hermes 预览窗格可直接看/录屏。

## 用法

```bash
# 1) 窗口服务（依赖安全节点发布 /safety/terrain_map + /safety/status_text）
bash /home/chj/pi_view_up.sh            # 重启窗口服务（会自动清旧进程与端口）

# 2) 录像（最稳的汇报方式：不依赖网络，直接播 MP4）
REC=30 bash /home/chj/pi_record.sh      # 录 30s → /home/chj/report_YYYYmmdd_HHMMSS.mp4

# 3) 相机（彩色 UVC 故障时用红外）
bash /home/chj/pi_cam_ir.sh             # 关彩色 / 开红外
bash /home/chj/pi_cam_fix.sh            # 相机无数据时一键重启
bash /home/chj/pi_stab.sh              # 查 WiFi 省电状态与话题速率
```

浏览器：`http://<pi-ip>:8080/`（F11 全屏录屏）

## 已知问题

- **彩色流无帧（UVC 通道故障）**：节点已注册 `/camera/color/image_raw`、日志 `color is started`，
  但 `topic hz`/单帧 `echo` 取不到；降分辨率帧率（320x240@15）与拔插 USB 均无效 ⇒ 用**红外**替代。
- **WiFi 偶发掉线**：`Power save: on` 是常见元凶，`sudo iw dev wlan0 set power_save off` 可显著改善。
- **录像时长偏差**：容器按 12fps 写，实际渲染约 9fps ⇒ 播放略快（约 1.3×），待改为按实测帧率写入。

## 依赖

- 相机：`~/astra_ws`（astra_camera）；感知：`~/mdw_ws`（mechdog_navigation_ros）
- Python：numpy / cv2 / PIL（Pi 上已具备）
- 环境：`ROS_DOMAIN_ID=42`
