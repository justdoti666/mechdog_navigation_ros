#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hidden_probe.py — A 批「材质指纹对照实验」采集器（2026-10-15）

作用
----
在 Pi（或任何跑着 `safety_node` 的机器）上，把**三层传感器对同一块材质的反应**按同一时刻记下来：

  · 超声层   ← `/ultrasonic`（`mechdog_ultrasonic/UltrasonicArray`：三路前向 + 底部，含 valid 标志）
  · 深度层   ← `/safety/terrain_grid`（`mono8` 2.5D 栅格，取值 = CellFlag 0..4）
  · 交叉层   ← `/fusion_result`（节点自己算的 `hidden_obstacle` / `hidden_dir` / `hidden_dist_m` / `hidden_slow`）

为什么要第三层：**不要用本脚本重新实现判据**。`hidden_obstacle` 是节点（`sensor_fusion.cpp`
`evaluate_hidden_obstacle_dir`）按 窗 [0.90,3.00]m / 距离带 ±0.25m / 扇区 ±25° 算出来的**真值**；
本脚本只做两件事：① 把它与原始读数**按帧配对**存档；② 顺带用 `terrain_grid` 复算"该距离带内
有没有禁行格"（`block` 计数），作为**深度层的独立证据**（= 辅助列，不是判据本体）。

口径（与节点一致，改了要同步）
------------------------------
· 栅格几何：`wx = min_x + (c+0.5)*cell`，`wy = -y_half + (r+0.5)*cell`；`cell=0.05`、
  `min_x=0.6`、`y_half=2.5`（`heightmap_2d5.cpp:20-24` / `HeightMap25Config`）。
· 禁行格 = `1=ObstacleUp / 2=CliffDown / 4=TooSteep`（与 `config.h HiddenObstacleConfig` 同口径；
  `0=Unknown / 3=Traversable` 不算"解释"）。
· 方向取角：left `+yaw_mag`、center `0`、right `-yaw_mag`（`+y=左`，`yaw_mag` 默认 30°）。

用法（Pi 上）
-------------
    source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=42
    python3 hidden_probe.py --preflight                 # 前置检查：链路/超声/深度是否真在
    python3 hidden_probe.py --label label.json --out raw.jsonl --secs 20
    python3 hidden_probe.py --label label.json --out raw.jsonl --symbols 5   # 标 5 个测点后退出

`label.json`（现场每换一次材质/距离就改，**不用重启**；示例见同目录
`材质对照实验协议_2026-10-15.md` §4）：
    {"material": "acrylic_mirror", "material_cn": "亚克力镜面板",
     "dist_m": 1.00, "height_m": 0.35, "angle_deg": 0, "note": "迎光面朝狗"}

运行中按键：`空格` = 打一个 SYMBOL 标记（把最近一次采样标成"标定点"）· `q` = 收工。
每个采样点写一行 JSON（JSONL），字段见 §输出。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
except Exception as exc:  # pragma: no cover - 无 ROS 环境时给出人话
    print(f"[hidden_probe] 需要 ROS2 环境（source /opt/ros/jazzy/setup.bash）: {exc}", file=sys.stderr)
    raise

try:
    from mechdog_ultrasonic.msg import UltrasonicArray
    HAVE_ULTRA_MSG = True
except Exception:  # 消息包未 source 时降级：只记深度+交叉层
    UltrasonicArray = None
    HAVE_ULTRA_MSG = False

# ---- 与节点一致的口径常量（核心 config.h / heightmap_2d5.cpp）----
CELL = 0.05
MIN_X = 0.60
Y_HALF = 2.50
BLOCKED_FLAGS = (1, 2, 4)          # ObstacleUp / CliffDown / TooSteep
FLAG_NAME = {0: "Unknown", 1: "ObstacleUp", 2: "CliffDown", 3: "Traversable", 4: "TooSteep"}
MIN_CMP_M, MAX_CMP_M, BAND_M, SECTOR_HALF_DEG = 0.90, 3.00, 0.25, 25.0

# 配对新鲜度上限（秒）：节点 5Hz 发 /fusion_result、超声 20Hz ⇒ 3s 内仍视为"同一时刻的现场"
FUSION_MAX_AGE_S = 3.0
ULTRA_MAX_AGE_S = 3.0

FUSION_KEYS = ("hidden_obstacle", "hidden_dir", "hidden_dist_m", "hidden_slow",
               "degraded", "cliff", "action", "min_fwd_m", "vx", "wz", "environment", "timestamp")


def dirs_of(ultra_msg):
    """→ {dir: (distance_m, valid)}；无消息时返回空 dict。"""
    if ultra_msg is None:
        return {}
    out = {}
    for name, d_cm, ok in (("left", ultra_msg.front_left_cm, ultra_msg.front_left_valid),
                           ("center", ultra_msg.front_center_cm, ultra_msg.front_center_valid),
                           ("right", ultra_msg.front_right_cm, ultra_msg.front_right_valid)):
        m = None if (d_cm is None or not math.isfinite(d_cm)) else d_cm / 100.0
        out[name] = (m, bool(ok))
    return out


class HiddenProbe(Node):
    def __init__(self, args):
        super().__init__("hidden_probe")
        self.args = args
        self.fusion = None            # 最近一条 /fusion_result
        self.ultra = None             # 最近一条 /ultrasonic
        self.terrain = None           # (stamp, w, h, bytes) 最近一张 /safety/terrain_grid
        self.n_fusion = 0
        self.n_ultra = 0
        self.n_terrain = 0
        self.t_fusion = None
        self.t_ultra = None
        self.t_terrain = None
        self.create_subscription(String, args.fusion_topic, self.on_fusion, 10)
        if HAVE_ULTRA_MSG:
            self.create_subscription(UltrasonicArray, args.ultra_topic, self.on_ultra,
                                     qos_profile_sensor_data)
        if args.terrain:
            self.create_subscription(Image, args.terrain_topic, self.on_terrain, 1)

    # ---- 回调：只存"最近一帧"，配对在采样时刻做 ----
    def on_fusion(self, msg: String):
        self.fusion = msg.data
        self.n_fusion += 1
        self.t_fusion = time.time()

    def on_ultra(self, msg):
        self.ultra = msg
        self.n_ultra += 1
        self.t_ultra = time.time()

    def on_terrain(self, msg: Image):
        self.terrain = (msg.width, msg.height, bytes(msg.data), msg.encoding)
        self.n_terrain += 1
        self.t_terrain = time.time()

    # ---- 深度层辅助列：复算某方向"距离带内禁行格数"（与节点同几何）----
    def band_stats(self, dist_m: float, yaw_deg: float):
        if self.terrain is None or dist_m is None:
            return None
        w, h, data, enc = self.terrain
        if enc != "mono8" or len(data) < w * h:
            return None
        lo, hi = dist_m - BAND_M, dist_m + BAND_M
        half = math.radians(SECTOR_HALF_DEG)
        yaw = math.radians(yaw_deg)
        cells = block = trav = unknown = up = down = steep = 0
        for r in range(h):
            wy = -Y_HALF + (r + 0.5) * CELL
            for c in range(w):
                wx = MIN_X + (c + 0.5) * CELL
                rad = math.hypot(wx, wy)
                if rad < lo or rad > hi:
                    continue
                d = math.atan2(wy, wx) - yaw
                while d > math.pi:
                    d -= 2 * math.pi
                while d < -math.pi:
                    d += 2 * math.pi
                if abs(d) > half:
                    continue
                cells += 1
                f = data[r * w + c]
                if f == 1:
                    up += 1
                elif f == 2:
                    down += 1
                elif f == 4:
                    steep += 1
                elif f == 3:
                    trav += 1
                else:
                    unknown += 1
                if f in BLOCKED_FLAGS:
                    block += 1
        return {"cells_in_band": cells, "blockers_in_band": block,
                "traversable_in_band": trav, "unknown_in_band": unknown,
                "up": up, "down": down, "steep": steep}

    def sample(self, label: dict, yaw_mag: float, marked: bool):
        rec = {
            "t_wall": round(time.time(), 3),
            "label": label,
            "marked": bool(marked),
            "counters": {"fusion": self.n_fusion, "ultra": self.n_ultra, "terrain": self.n_terrain},
            "age_s": {
                "fusion": None if self.t_fusion is None else round(time.time() - self.t_fusion, 3),
                "ultra": None if self.t_ultra is None else round(time.time() - self.t_ultra, 3),
                "terrain": None if self.t_terrain is None else round(time.time() - self.t_terrain, 3),
            },
            "ultra_dirs": {},
            "ultra_bottom_cm": None,
        }
        # 超声层（原始读数，不加工）
        if self.ultra is not None:
            for k, (m, ok) in dirs_of(self.ultra).items():
                rec["ultra_dirs"][k] = {"dist_m": None if m is None else round(m, 4), "valid": ok}
            try:
                bc = self.ultra.bottom_cm
                rec["ultra_bottom_cm"] = None if bc is None or not math.isfinite(bc) else round(bc, 2)
            except Exception:
                pass
            rec["ultra_seq"] = int(getattr(self.ultra, "seq", -1))
        # 交叉层（节点真值）
        node = {}
        if self.fusion:
            try:
                j = json.loads(self.fusion)
                node = {k: j.get(k) for k in FUSION_KEYS if k in j}
            except Exception:
                node = {"_parse_error": self.fusion[:120]}
        rec["node"] = node
        # 新鲜度标记（2026-10-15 E2E 自检发现）：节点 5Hz、超声 20Hz，配对窗口取 3s；
        #   超过 = 这一条不可信 ⇒ 打 fresh=false，分析器默认剔除（不是丢弃，保留可追溯）。
        fa = rec["age_s"]["fusion"]
        ua = rec["age_s"]["ultra"]
        rec["fresh"] = bool(self.fusion) and (fa is not None and fa <= FUSION_MAX_AGE_S) \
            and (self.ultra is not None and ua is not None and ua <= ULTRA_MAX_AGE_S)
        # 深度层辅助列（每个方向）
        for d, yaw in (("left", +yaw_mag), ("center", 0.0), ("right", -yaw_mag)):
            u = rec["ultra_dirs"].get(d)
            if u and u["dist_m"] is not None and MIN_CMP_M <= u["dist_m"] <= MAX_CMP_M:
                st = self.band_stats(u["dist_m"], yaw)
                if st:
                    u["band"] = st
                    u["hidden_like"] = (u["valid"] and st["blockers_in_band"] == 0)
        return rec


def preflight(node: HiddenProbe, secs: float = 6.0):
    """前置检查：跑一次就能知道"今天能不能做材质实验"。"""
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.1)
    ok = True
    print("== 前置检查 ==")
    print(f"  /fusion_result : {node.n_fusion} 条 " + ("✓" if node.n_fusion > 0 else "✗ 节点没在跑?"))
    ok &= node.n_fusion > 0
    if not HAVE_ULTRA_MSG:
        print("  /ultrasonic    : ✗ 消息包未 source（colcon 后 source install/setup.bash）")
        ok = False
    elif node.n_ultra == 0:
        print("  /ultrasonic    : ✗ 一条都没收到 ⇒ 超声链路未上（实验的「有回波」这一列无从谈起）")
        ok = False
    else:
        d = dirs_of(node.ultra)
        s = ", ".join(f"{k}={'%.3fm' % v[0] if v[0] is not None else 'NA'}/{'有效' if v[1] else '无效'}"
                      for k, v in d.items())
        print(f"  /ultrasonic    : {node.n_ultra} 条 ✓  latest: {s}")
    if node.args.terrain:
        if node.n_terrain == 0:
            print("  terrain_grid   : ✗ 没收到（相机/平面? 看节点日志 DEPTH GATE / NO PLANE）")
            ok = False
        else:
            w, h, data, enc = node.terrain
            from collections import Counter
            cnt = Counter(data)
            tot = w * h
            share = {FLAG_NAME.get(k, k): f"{v*100.0/tot:.1f}%" for k, v in sorted(cnt.items())}
            print(f"  terrain_grid   : {w}x{h} {enc} ✓  分布={share}")
            if cnt.get(3, 0) + cnt.get(1, 0) + cnt.get(2, 0) + cnt.get(4, 0) == 0:
                print("      ⚠ 全是 Unknown ⇒ 本轮深度没有地面样本（回放灌帧没发 camera_info 也会这样）")
    print(f"== 结论: {'可以开始实验' if ok else '前置不满足，先解决上面 ✗ 的项'} ==")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument("--label", default=None, help="测点标签 JSON（现场可随时改，不必重启）")
    ap.add_argument("--out", default="hidden_probe.jsonl", help="输出 JSONL")
    ap.add_argument("--hz", type=float, default=2.0, help="采样频率（默认 2Hz）")
    ap.add_argument("--secs", type=float, default=None, help="总时长（默认不限，Ctrl-C/q 结束）")
    ap.add_argument("--symbols", type=int, default=None, help="标够 N 个测点后自动退出")
    ap.add_argument("--terrain", action="store_true", help="订阅 /safety/terrain_grid 并复算禁行格")
    ap.add_argument("--preflight", action="store_true", help="只做前置检查")
    ap.add_argument("--yaw-mag", type=float, default=30.0, help="左右方向取角幅度（默认 30°，同 layout）")
    ap.add_argument("--fusion-topic", default="/fusion_result")
    ap.add_argument("--ultra-topic", default="/ultrasonic")
    ap.add_argument("--terrain-topic", default="/safety/terrain_grid")
    args = ap.parse_args()

    def load_label():
        if not args.label:
            return {"material": "unknown", "dist_m": None}
        try:
            with open(args.label, encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            print(f"[hidden_probe] 读 label 失败({e})，用占位标签", file=sys.stderr)
            return {"material": "unknown", "dist_m": None}

    rclpy.init()
    node = HiddenProbe(args)
    try:
        if args.preflight:
            return 0 if preflight(node) else 3

        print(f"[hidden_probe] 采样 {args.hz}Hz → {args.out}   "
              f"(label={args.label or '<none>'} terrain={'on' if args.terrain else 'off'})")
        period = 1.0 / max(args.hz, 0.1)
        t0 = time.time()
        n_sym = 0
        mark_next = [False]          # 主循环置位；采样时消费
        # 预热：先转 0.6s，把订阅建立/首帧到达的时延吃掉（否则开头几条会 age_s 偏大被判不新鲜）
        t_warm = time.time()
        while rclpy.ok() and (time.time() - t_warm) < 0.6:
            rclpy.spin_once(node, timeout_sec=0.05)
        with open(args.out, "a", encoding="utf-8") as fo:
            while rclpy.ok():
                # 键盘（可选，非阻塞）
                if sys.stdin.isatty():
                    import select
                    r, _, _ = select.select([sys.stdin], [], [], 0)
                    if r:
                        ch = sys.stdin.readline().strip().lower()
                        if ch == "q":
                            break
                        mark_next[0] = True
                label = load_label()          # ← 每轮重读：现场改 label.json 立即生效
                rec = node.sample(label, args.yaw_mag, mark_next[0])
                if mark_next[0]:
                    n_sym += 1
                    mark_next[0] = False
                fo.write(json.dumps(rec, ensure_ascii=False) + "\n")
                fo.flush()
                # 现场一行摘要（便于肉眼确认"这一秒发生了什么"）
                nd = rec.get("node", {})
                u = rec.get("ultra_dirs", {})
                uc = u.get("center", {})
                band = uc.get("band") if isinstance(uc, dict) else None
                print(f"  [{n_sym:02d}{'*' if rec['marked'] else ' '}] "
                      f"HIDDEN={'Y' if nd.get('hidden_obstacle') else 'n'}"
                      f" dir={nd.get('hidden_dir','')}@{nd.get('hidden_dist_m')}"
                      f" | 超声 center={uc.get('dist_m')}m/{'ok' if uc.get('valid') else 'NA'}"
                      f" | 带内禁行格={band['blockers_in_band'] if band else 'n/a'}"
                      f"/{band['cells_in_band'] if band else 'n/a'}"
                      f" | slow={nd.get('hidden_slow')} action={nd.get('action')}"
                      f"{'' if rec.get('fresh') else ' [不新鲜,分析器会剔除]'}")
                if args.secs is not None and (time.time() - t0) >= args.secs:
                    break
                if args.symbols is not None and n_sym >= args.symbols:
                    break
                rclpy.spin_once(node, timeout_sec=period)
    except KeyboardInterrupt:
        print("\n[hidden_probe] Ctrl-C 收工")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f"[hidden_probe] 已写 {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
