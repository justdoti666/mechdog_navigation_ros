#!/usr/bin/env python3
"""L0 速度仲裁层 A/B 探针 (safety_node, arbiter_enable true/false)

用法:
  python3 arbiter_probe.py <mode> [out_topic] [desired_topic]
    mode = true | false      (与 arbiter_enable 一致)
    out_topic     默认 /l0/out
    desired_topic 默认 /l0/desired

判据 (mode=true 才判, mode=false 只记录):
  A 期望 (0.15, 0.10)  —— 未越界 ⇒ 每帧 |vx| <= 0.15, 且出现过 0.15
  B 期望 (0.50, 0.20)  —— 越界   ⇒ 每帧 |vx| <= 0.20, 且出现过 0.20; |wz| <= 0.20
  C 停止发布           —— >300ms  ⇒ 输出零速
  （融合动作在 sim 下会逐帧抖动 —— SLOW/TURN 把速度压到 0.10/0 是**设计行为**,
     不是失败; 因此判"上限 + 出现过"而不是"恒定值"）

退出码: 0 = PASS, 1 = FAIL
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MODE = sys.argv[1] if len(sys.argv) > 1 else "true"
OUT_TOPIC = sys.argv[2] if len(sys.argv) > 2 else "/l0/out"
DES_TOPIC = sys.argv[3] if len(sys.argv) > 3 else "/l0/desired"
CSV_PATH = f"/tmp/arbiter_probe_{MODE}_samples.csv"


class Probe(Node):
    def __init__(self):
        super().__init__("arbiter_probe")
        self.pub = self.create_publisher(Twist, DES_TOPIC, 10)
        self.sub = self.create_subscription(Twist, OUT_TOPIC, self.cb, 20)
        self.samples = []
        self.phases = []

    def cb(self, msg):
        self.samples.append((time.time(), msg.linear.x, msg.angular.z))

    def phase(self, name, secs, vx, wz):
        t0 = time.time()
        while time.time() - t0 < secs:
            m = Twist()
            m.linear.x = vx
            m.angular.z = wz
            self.pub.publish(m)
            # 必须在本循环里 spin, 否则订阅回调只在相位之外才跑 (窗口内 0 样本)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.08)
        self.phases.append((name, t0, time.time()))


def window(samples, t0, t1, skip=0.6):
    return [(v, w) for (t, v, w) in samples if t0 + skip <= t <= t1]


def main():
    rclpy.init()
    p = Probe()
    t_end = time.time() + 5.0          # 等 warmup + 首轮融合
    while time.time() < t_end:
        rclpy.spin_once(p, timeout_sec=0.1)

    p.phase("A", 2.5, 0.15, 0.10)
    p.phase("B", 2.5, 0.50, 0.20)
    p.phase("C", 0.6, 0.00, 0.00)      # 之后彻底停发 ⇒ 触发 300ms 超时
    t_end = time.time() + 1.5
    while time.time() < t_end:
        rclpy.spin_once(p, timeout_sec=0.1)

    with open(CSV_PATH, "w", encoding="utf-8") as f:
        f.write("t,vx,wz\n")
        for t, v, w in p.samples:
            f.write(f"{t:.3f},{v:.4f},{w:.4f}\n")

    print(f"mode={MODE} samples={len(p.samples)} (csv: {CSV_PATH})")
    ok = True
    for name, t0, t1 in p.phases:
        vals = window(p.samples, t0, t1)
        if not vals:
            if name != "C":
                print(f"FAIL {name}: NO SAMPLES")
                ok = False
            else:
                print("C: NO SAMPLES (预期: 用 C_timeout 判)")
            continue
        vs = [v for v, _ in vals]
        ws = [w for _, w in vals]
        print(f"{name}: vx[min={min(vs):.3f} max={max(vs):.3f}] "
              f"wz[min={min(ws):.3f} max={max(ws):.3f}] n={len(vals)}")

    t_c = p.phases[-1][2] + 0.6
    post = [(v, w) for (t, v, w) in p.samples if t >= t_c]
    post_max = max((abs(v) for v, _ in post), default=None)
    print(f"C_timeout: post-stop max|vx|={post_max if post_max is None else round(post_max, 3)} n={len(post)}")

    if MODE == "true":
        for name, des_v, des_w in (("A", 0.15, 0.10), ("B", 0.50, 0.20)):
            t0 = next(t0 for n, t0, _ in p.phases if n == name)
            t1 = next(t1 for n, _, t1 in p.phases if n == name)
            vals = window(p.samples, t0, t1)
            if not vals:
                continue   # 已在上面记账
            cap = min(des_v, 0.20)
            if any(abs(v) > cap + 1e-6 for v, _ in vals):
                print(f"FAIL {name}: 违反只减不增 (上限 {cap:.2f})")
                ok = False
            if any(abs(w) > des_w + 1e-6 for _, w in vals):
                print(f"FAIL {name}: 角速度被放大 (上限 {des_w:.2f})")
                ok = False
            if not any(abs(abs(v) - cap) < 1e-6 for v, _ in vals):
                print(f"FAIL {name}: 从未达到期望上限 {cap:.2f} (期望未被采用?)")
                ok = False
        if post_max is None or post_max > 1e-6:
            print("FAIL C: 期望速度超时 >300ms 应输出零速")
            ok = False

    print("RESULT:", "PASS" if ok else "FAIL")
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
