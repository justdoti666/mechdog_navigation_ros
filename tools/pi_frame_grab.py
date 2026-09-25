#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pi_frame_grab.py —— 抓一批真机帧存盘(供离线重算/同帧比对)

用途:
  1) 离线重算基线(#2): 与节点日志"同帧逐字段一致"
  2) cell vs RANSAC 对照(决策 1 的补充证据)
  3) 装机后复测: 同一工具、同一口径, 新旧对比

产出(默认 /home/chj/frames_<标签>/):
  depth_000.npy     16UC1 原始深度(mm), 640x480
  caminfo.json      K/D/尺寸
  params.json       抓帧时的节点参数(从命令行传)
  stamps.txt        每帧 header.stamp + 本机时刻 + 有效像素占比
  node_line.txt     同批时刻节点日志里的 感知: 行(若可读)

用法(在 Pi 上):
  export ROS_DOMAIN_ID=42
  source /opt/ros/jazzy/setup.bash; source ~/mdw_ws/install/setup.bash
  python3 pi_frame_grab.py 5 平地
    参数: 第1个 = 抓几帧(默认 5); 第2个 = 标签(默认 grab)
"""
import sys, os, json, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

N = int(sys.argv[1]) if len(sys.argv) > 1 else 5
TAG = sys.argv[2] if len(sys.argv) > 2 else "grab"
OUT = f"/home/chj/frames_{TAG}"
os.makedirs(OUT, exist_ok=True)

rclpy.init()
node = Node("frame_grab")
q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST)
state = {"ci": None, "n": 0, "t0": None}

def on_ci(m):
    state["ci"] = {
        "width": m.width, "height": m.height,
        "k": list(m.k), "d": list(m.d),
        "distortion_model": m.distortion_model,
        "frame_id": m.header.frame_id,
    }

def on_depth(m):
    if state["n"] >= N:
        return
    a = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width).copy()
    valid = int(((a >= 600) & (a <= 8000)).sum())
    ratio = valid / float(a.size)
    idx = state["n"]
    np.save(os.path.join(OUT, f"depth_{idx:03d}.npy"), a)
    with open(os.path.join(OUT, "stamps.txt"), "a", encoding="utf-8") as f:
        f.write(f"depth_{idx:03d} stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d} "
                f"wall={time.time():.6f} valid_px={valid} valid_ratio={ratio:.4f} "
                f"w={m.width} h={m.height} enc={m.encoding} step={m.step}\n")
    if state["t0"] is None:
        state["t0"] = time.time()
    state["n"] += 1
    print(f"  抓 {idx+1}/{N}  valid={valid} ({ratio*100:.1f}%)", flush=True)

node.create_subscription(CameraInfo, "/camera/depth/camera_info", on_ci, q)
node.create_subscription(Image, "/camera/depth/image_raw", on_depth, q)

t_end = time.time() + 40
while time.time() < t_end and state["n"] < N:
    rclpy.spin_once(node, timeout_sec=0.2)

if state["ci"] is None:
    # 兜底: 话题名不同就试 IR/彩色
    print("!! 没收到 camera_info, 请确认话题名", flush=True)
else:
    with open(os.path.join(OUT, "caminfo.json"), "w", encoding="utf-8") as f:
        json.dump(state["ci"], f, ensure_ascii=False, indent=2)

# 记录抓帧时的节点参数(供离线复现)
params = {
    "camera_height_m": None, "cloud_z": None, "cloud_pitch_rad": None,
    "cloud_roll_rad": None, "cloud_yaw_rad": None,
    "ground_fit_method": "cell", "note": "从命令行参数抄填; 见抓帧日志",
}
with open(os.path.join(OUT, "params.json"), "w", encoding="utf-8") as f:
    json.dump(params, f, ensure_ascii=False, indent=2)

# 顺带捞节点日志里的 感知: 行(同批证据)
try:
    with open("/home/chj/report_node.log", "r", encoding="utf-8", errors="replace") as f:
        lines = [l for l in f if "感知:" in l][-3:]
    with open(os.path.join(OUT, "node_line.txt"), "w", encoding="utf-8") as f:
        f.writelines(lines)
except Exception as e:
    print("捞节点日志失败:", e)

print(f"完成: {state['n']} 帧 -> {OUT}", flush=True)
rclpy.shutdown()
