#!/usr/bin/env python3
"""深度话题健康探针: 30s 窗口内 帧率 / 有效像素% / 深度中位 / 空行统计。

修复记录 (v2.9.21, T-C2 / 复审批 M-6): 旧版 numpy 解析未用 msg.step —— 发布端一旦
带行填充 (step > width*2) 就会 reshape 崩溃; 现按 step 解析后裁到 width 列。
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

DUR = 30.0


def parse_depth(msg):
    """按 step 解析 16UC1 深度图, 裁剪到 width 列 (兼容行填充)。"""
    step = msg.step if msg.step else msg.width * 2
    rows = msg.height
    buf = np.frombuffer(msg.data, dtype=np.uint16, count=rows * (step // 2))
    return buf.reshape(rows, step // 2)[:, :msg.width]


def main():
    rclpy.init()
    n = Node("depth_health_probe")
    q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST)
    rec = []
    empty_rows = []

    def cb(m):
        a = parse_depth(m)
        v = a[a > 0]
        vr = 100.0 * v.size / a.size
        med = int(np.median(v)) if v.size else 0
        rec.append((time.time(), vr, med))
        empty_rows.append(int(np.sum(np.all(a == 0, axis=1))))

    n.create_subscription(Image, "/camera/depth/image_raw", cb, q)
    t0 = time.time()
    while time.time() - t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.05)
    rclpy.shutdown()

    if not rec:
        print("未取到任何帧")
        return
    arr = np.array([r[1] for r in rec])
    meds = np.array([r[2] for r in rec])
    span = rec[-1][0] - rec[0][0]
    print("窗口 %.1fs 内收到 %d 帧 ⇒ %.1f Hz" % (span, len(rec), len(rec) / max(span, 0.1)))
    print("有效像素%%: 最小 %.1f / 中位 %.1f / 最大 %.1f" % (arr.min(), np.median(arr), arr.max()))
    print("深度中位 mm: 最小 %d / 中位 %d / 最大 %d" % (meds.min(), int(np.median(meds)), meds.max()))
    print("★ valid<25%%(会被守门拦) 的帧: %d / %d = %.1f%%"
          % ((arr < 25).sum(), len(arr), 100.0 * (arr < 25).sum() / len(arr)))
    print("★ valid==0%%(全零帧)      : %d / %d = %.1f%%"
          % ((arr == 0).sum(), len(arr), 100.0 * (arr == 0).sum() / len(arr)))
    print("★ 有全空行的帧            : %d / %d"
          % (int(np.sum(np.array(empty_rows) > 0)), len(rec)))
    worst = sorted(range(len(rec)), key=lambda i: rec[i][1])[:5]
    print("最差 5 帧: " + " | ".join("%.1f%%/med=%d" % (rec[i][1], rec[i][2]) for i in worst))


if __name__ == "__main__":
    main()
