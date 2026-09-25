#!/usr/bin/env python3
# 回放 raw 深度到 /camera/depth/image_raw (用于"同一帧"节点 vs 基准 对照)
#
# v2 用法(2026-09-24 扩展): 支持"抓帧批次目录"
#   单文件(旧): python3 pi_replay.py /home/chj/real_depth.raw 20      # 回放该帧 20 秒
#   批次(新)  : python3 pi_replay.py /home/chj/frames_平地 10 1       # 目录里每帧@10Hz回放1遍
#     批次目录由 pi_frame_grab.py 产出: depth_*.npy + caminfo.json + stamps.txt
#     回放时**沿用原 stamp**(若 stamps.txt 有), 于是节点日志里的 感知: 行可与离线逐帧对齐。
import sys, os, glob, json
import numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

arg1 = sys.argv[1] if len(sys.argv) > 1 else '/home/chj/real_depth.raw'
BATCH = os.path.isdir(arg1)
if BATCH:
    HZ = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    REPEAT = int(sys.argv[3]) if len(sys.argv) > 3 else 1
    files = sorted(glob.glob(os.path.join(arg1, 'depth_*.npy')))
    if not files:
        print(f'!! {arg1} 里没有 depth_*.npy'); sys.exit(1)
    stamps = {}
    sp = os.path.join(arg1, 'stamps.txt')
    if os.path.exists(sp):
        for ln in open(sp, encoding='utf-8'):
            p = ln.split()
            if p and p[0].startswith('depth_') and any(x.startswith('stamp=') for x in p):
                s, ns = [x for x in p if x.startswith('stamp=')][0][6:].split('.')
                stamps[p[0]] = (int(s), int(ns))
    order = []          # [(ndarray, stamp_key, repeat_idx)]
    for f in files:
        a = np.load(f)
        key = os.path.basename(f)[:-4]
        for k in range(REPEAT):
            order.append((a, key, k))
    _cip = os.path.join(arg1, 'caminfo.json')
    CI = json.load(open(_cip, encoding='utf-8')) if os.path.exists(_cip) else {}
    print(f'批次回放: {len(files)} 帧 × {REPEAT} 遍 @ {HZ}Hz, 目录 {arg1}', flush=True)
else:
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    raw = np.fromfile(arg1, dtype=np.uint16).reshape(480, 640)

class P(Node):
    def __init__(self):
        super().__init__('frame_replay')
        self.pub = self.create_publisher(Image, '/camera/depth/image_raw', 5)
        self.inf = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 5)
        self.n = 0
        self.i = 0
        if BATCH:
            self.t = self.create_timer(1.0 / max(0.1, HZ), self.cb_batch)
        else:
            self.t = self.create_timer(0.1, self.cb)   # 10 Hz
    def cb(self):
        m = Image(); m.height = 480; m.width = 640
        m.encoding = '16UC1'; m.is_bigendian = 0; m.step = 640 * 2
        m.data = raw.tobytes()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'camera_depth_optical_frame'
        self.pub.publish(m)
        ci = CameraInfo(); ci.height = 480; ci.width = 640
        ci.k = [570.3422047415297129, 0.0, 319.5, 0.0, 570.3422047415297129, 239.5, 0.0, 0.0, 1.0]
        ci.header = m.header
        self.inf.publish(ci)
        self.n += 1
    def cb_batch(self):
        if self.i >= len(order):
            self.t.cancel()
            return
        a, key, k = order[self.i]
        h, w = a.shape
        m = Image(); m.height = int(h); m.width = int(w)
        m.encoding = '16UC1'; m.is_bigendian = 0; m.step = int(w * 2)
        m.data = a.astype(np.uint16).tobytes()
        m.header.frame_id = 'camera_depth_optical_frame'
        if key in stamps:
            m.header.stamp.sec, m.header.stamp.nanosec = stamps[key]
        else:
            m.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(m)
        if k == 0:
            ci = CameraInfo(); ci.header = m.header; ci.height = int(h); ci.width = int(w)
            ci.distortion_model = CI.get('distortion_model', 'plumb_bob')
            ci.k = [float(x) for x in CI.get('k')] if CI.get('k') else \
                   [570.3422047415297129, 0.0, 319.5, 0.0, 570.3422047415297129, 239.5, 0.0, 0.0, 1.0]
            ci.d = [float(x) for x in CI.get('d', [])]
            ci.p = ci.k[:3] + [0.0] + ci.k[3:6] + [0.0] + [0.0, 0.0, 1.0, 0.0]
            self.inf.publish(ci)
        print(f"  发 {key} ({self.i + 1}/{len(order)}) "
              f"{'原stamp' if key in stamps else '新stamp'} valid={int(((a >= 600) & (a <= 8000)).sum())}",
              flush=True)
        self.i += 1
        self.n += 1

rclpy.init(); n = P()
import time
t0 = time.time()
hard_cap = (len(order) / max(0.1, HZ) + 10.0) if BATCH else secs
while rclpy.ok() and time.time() - t0 < hard_cap:
    rclpy.spin_once(n, timeout_sec=0.05)
    if BATCH and n.i >= len(order):
        time.sleep(0.5)
        break
print(f"REPLAY_DONE frames={n.n}")
if BATCH:
    print(f"→ 现在对比: grep -a '感知:' /home/chj/report_node.log | tail -{len(order) // 2 + 2}")

