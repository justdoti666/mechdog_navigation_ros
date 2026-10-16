#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hidden_summarize.py — A 批材质实验：原始 JSONL → 「材质 × 传感器」指纹矩阵（2026-10-15）

输入 = `hidden_probe.py` 的 JSONL（每行一个采样点；每行自带当时生效的 label）。
输出 = ① 逐测点明细表 ② 材质×传感器矩阵（超声/深度/交叉层）③ 判读建议。

判读口径（**都写成常量，改了要在协议文档同步**）
------------------------------------------------
· 超声"有回波" = `ultra_dirs[d].valid` 且 `dist_m ∈ [MIN_CMP_M, MAX_CMP_M]`；
  "距离准" = 均值与标签距离差 ≤ `DIST_TOL_M`。
· 深度"有障碍" = 该方向距离带内 `blockers_in_band > 0`（栅格层复算，禁行格=1/2/4）。
  深度"看穿" = 带内有样本但**零禁行格**（`cells_in_band>0 & blockers==0`）——玻璃/镜面的签名。
· 交叉层 = 节点 `hidden_obstacle`（窗/带/扇区由节点自己算，本脚本不重算）。
· 逐点"命中"要求该点内 ≥ `MIN_HITS` 条采样支持，避免单帧抖动当结论。

用法：
    python3 hidden_summarize.py --in raw.jsonl --out 矩阵.md
    python3 hidden_summarize.py --in raw.jsonl --only-marked      # 只用打标的测点
"""
from __future__ import annotations

import argparse
import json
import math
import statistics as st
import sys
from collections import defaultdict

MIN_CMP_M, MAX_CMP_M = 0.90, 3.00
DIST_TOL_M = 0.25
MIN_HITS = 3
FUSION_STALE_WARN_S = 2.0


def load(path: str):
    recs = []
    with open(path, encoding="utf-8") as f:
        for ln, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                recs.append(json.loads(line))
            except Exception as e:
                print(f"  [warn] 第 {ln} 行不是 JSON，跳过: {e}", file=sys.stderr)
    return recs


def key_of(rec):
    lab = rec.get("label") or {}
    return (lab.get("material", "unknown"), lab.get("material_cn", ""),
            lab.get("dist_m"), lab.get("height_m"))


def summarize_dirs(recs, d):
    """单方向：超声 + 深度带内复算 的统计。"""
    dists, n_valid = [], 0
    cells, blocks, block_hist = [], [], defaultdict(int)
    for r in recs:
        u = (r.get("ultra_dirs") or {}).get(d)
        if not u:
            continue
        if u.get("valid") and u.get("dist_m") is not None:
            n_valid += 1
            dists.append(u["dist_m"])
        b = u.get("band")
        if b:
            cells.append(b["cells_in_band"])
            blocks.append(b["blockers_in_band"])
            block_hist[b["blockers_in_band"]] += 1
    out = {
        "n": len(recs), "n_valid": n_valid,
        "valid_pct": (100.0 * n_valid / len(recs)) if recs else 0.0,
        "dist_mean": st.mean(dists) if dists else None,
        "dist_sd": (st.pstdev(dists) if len(dists) > 1 else 0.0) if dists else None,
        "dist_min": min(dists) if dists else None,
        "dist_max": max(dists) if dists else None,
        "cells_in_band_mean": st.mean(cells) if cells else None,
        "blockers_mean": st.mean(blocks) if blocks else None,
        "blockers_zero_pct": (100.0 * block_hist[0] / len(blocks)) if blocks else None,
        "block_hist": dict(sorted(block_hist.items())),
    }
    return out


def summarize_node(recs):
    n = len(recs)
    hid = [r for r in recs if (r.get("node") or {}).get("hidden_obstacle")]
    dirs = defaultdict(int)
    for r in hid:
        dirs[(r.get("node") or {}).get("hidden_dir", "")] += 1
    dists = [r["node"].get("hidden_dist_m") for r in hid if isinstance(r["node"].get("hidden_dist_m"), (int, float))]
    stale = [r for r in recs if (r.get("age_s") or {}).get("fusion") is None
             or (r["age_s"]["fusion"] or 0) > FUSION_STALE_WARN_S]
    return {
        "n": n,
        "hidden_pct": (100.0 * len(hid) / n) if n else 0.0,
        "hidden_dirs": dict(dirs),
        "hidden_dist_mean": st.mean(dists) if dists else None,
        "slow_pct": 100.0 * sum(1 for r in recs if (r.get("node") or {}).get("hidden_slow")) / n if n else 0.0,
        "degraded_pct": 100.0 * sum(1 for r in recs if (r.get("node") or {}).get("degraded")) / n if n else 0.0,
        "n_stale": len(stale),
    }


def fmt(v, spec="{:.3f}"):
    if v is None:
        return "—"
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return "—"
    try:
        return spec.format(v)
    except Exception:
        return str(v)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--out", dest="out", default=None)
    ap.add_argument("--only-marked", action="store_true", help="只用 marked=true 的采样点")
    ap.add_argument("--keep-stale", action="store_true",
                    help="默认剔除 fresh=false（配对超时）的采样点；此开关可保留（调试用）")
    ap.add_argument("--dir", default="center", choices=["center", "left", "right"],
                    help="矩阵用哪个方向（默认正前）")
    args = ap.parse_args()

    recs_all = load(args.inp)
    n_stale = sum(1 for r in recs_all if r.get("fresh") is False)
    recs = recs_all if args.keep_stale else [r for r in recs_all if r.get("fresh") is not False]
    if args.only_marked:
        recs = [r for r in recs if r.get("marked")]
    if not recs:
        print("没有可用采样点（--only-marked 时请确认打过标；全部不新鲜时看节点是否在跑）")
        return 1

    groups = defaultdict(list)
    for r in recs:
        groups[key_of(r)].append(r)

    lines = []
    A = lines.append
    A("# A 批材质对照实验 · 指纹矩阵（自动生成）\n")
    A(f"- 原始数据：`{args.inp}`（{len(recs)} 个采样点，{len(groups)} 个测点）")
    A(f"- 矩阵方向：**{args.dir}**；判读常量：窗 [{MIN_CMP_M},{MAX_CMP_M}]m / 距离带 ±0.25m / 逐点 ≥{MIN_HITS} 条")
    if args.only_marked:
        A("- 过滤：仅 `marked=true` 的采样点")
    if n_stale:
        A(f"- 已剔除 **{n_stale}** 个不新鲜采样点（`fresh=false`：距最近 `/fusion_result` 或 `/ultrasonic` 超过 3s）"
          + ("；本次用 `--keep-stale` 保留" if args.keep_stale else ""))
    stale = sum(1 for r in recs if ((r.get("age_s") or {}).get("fusion") or 0) > FUSION_STALE_WARN_S)
    if stale:
        A(f"- ⚠ {stale} 个采样点距最近一次 `/fusion_result` 超过 {FUSION_STALE_WARN_S}s（节点可能没在跑）")

    A("\n## 1 逐测点明细\n")
    A("| 材质 | 标称距离(m) | 高度(m) | 点数 | 超声有效% | 超声均值(m) | 带内格数 | 带内禁行格(均值) | 零禁行% | 节点 hidden% | 方向 |")
    A("|---|---|---|---|---|---|---|---|---|---|---|")
    rows = []
    for k in sorted(groups, key=lambda x: (str(x[0]), (x[2] if x[2] is not None else -1))):
        mat, mat_cn, dist, height = k
        rs = groups[k]
        d = summarize_dirs(rs, args.dir)
        nd = summarize_node(rs)
        A(f"| {mat_cn or mat} | {fmt(dist, '{:.2f}')} | {fmt(height, '{:.2f}')} | {len(rs)} | "
          f"{fmt(d['valid_pct'], '{:.0f}')} | {fmt(d['dist_mean'])} | {fmt(d['cells_in_band_mean'], '{:.0f}')} | "
          f"{fmt(d['blockers_mean'], '{:.1f}')} | {fmt(d['blockers_zero_pct'], '{:.0f}')} | "
          f"{fmt(nd['hidden_pct'], '{:.0f}')} | {','.join(f'{k2}×{v}' for k2, v in nd['hidden_dirs'].items()) or '—'} |")
        rows.append((k, d, nd))

    A("\n## 2 材质 × 传感器 指纹矩阵\n")
    A("判据：**超声检出**=有效%≥50 且 |均值−标称|≤0.25m；**深度检出**=该方向距离带内平均有 >0.5 个禁行格"
      "（禁行格=ObstacleUp/CliffDown/TooSteep，与节点同口径）；**交叉层**=节点 hidden% ≥50%（=超声有、深度无）。"
      "`○`=检出 · `·`=未检出 · `?`=数据不足。\n")
    A("| 材质 | 距离(m) | 超声（声波） | 深度 2.5D（结构光） | 交叉层结果 |")
    A("|---|---|---|---|---|")
    matrix_rows = []
    SHOW_BLOCK = 0.5      # 深度"看得见"的判据：带内平均禁行格 > 0.5
    for k, d, nd in rows:
        mat, mat_cn, dist, height = k
        name = mat_cn or mat
        if d["n_valid"] >= MIN_HITS and d["dist_mean"] is not None and dist is not None and abs(d["dist_mean"] - dist) <= DIST_TOL_M:
            us = "○"
        elif d["n_valid"] >= MIN_HITS:
            us = "·"
        else:
            us = "?"
        bm = d["blockers_mean"]
        dp = "?" if bm is None else ("○" if bm > SHOW_BLOCK else "·")
        cross = "○" if nd["hidden_pct"] >= 50.0 else ("·" if nd["n"] >= MIN_HITS else "?")
        A(f"| {name} | {fmt(dist, '{:.2f}')} | {us} {fmt(d['dist_mean'])}m/{fmt(d['valid_pct'], '{:.0f}')}% | "
          f"{dp} 带内禁行格均值 {fmt(bm, '{:.1f}')} | {cross} {fmt(nd['hidden_pct'], '{:.0f}')}% |")
        matrix_rows.append((name, dist, us, dp, cross, d, nd))

    A("\n## 3 判读建议（自动草稿，人工确认后再进汇报）\n")
    pats = defaultdict(list)
    for name, dist, us, dp, cross, d, nd in matrix_rows:
        pats[(us, dp, cross)].append(f"{name}@{fmt(dist, '{:.2f}')}m")
    legend = {
        ("○", "·", "○"): "**核心签名**：超声有回波 / 深度零禁行 / 节点判隐身 ⇒ 交叉层检测成立的工况",
        ("○", "○", "○"): "超声与深度都报有、节点仍判隐身 ⇒ **需人工复核**：多半是「带内禁行格恰为 0」边界（材料临界）、"
                         "或深度看到的点在扇区/距离带之外",
        ("○", "○", "·"): "超声与深度**都看得见** ⇒ 不构成隐身（正常障碍，应走原有避障链路）",
        ("·", "·", "·"): "两者都看不见 ⇒ 超声也失效（吸声材质如布料/海绵，或超出量程）",
        ("○", "·", "·"): "超声有、深度零禁行，但**节点未判隐身** ⇒ 查窗/带/扇区（0.9–3.0m、±25°）或地形未注入",
        ("·", "○", "·"): "深度看得见、超声没回波 ⇒ 斜面/强吸声，或超声未对准",
    }
    for kk, items in sorted(pats.items(), key=lambda x: -len(x[1])):
        txt = legend.get(kk, "未归类组合，需人工判读")
        A(f"- 超声{kk[0]} / 深度{kk[1]} / 交叉层{kk[2]}（{len(items)} 点）：{txt}")
        A(f"  - {'、'.join(items)}")

    out_txt = "\n".join(lines) + "\n"
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(out_txt)
        print(f"已写 {args.out}")
    else:
        print(out_txt)

    # 控制台摘要（一眼看核心签名）
    core = pats.get(("○", "·", "○"), [])
    print(f"\n[摘要] 测点 {len(groups)} 个 / 采样 {len(recs)} 条；核心签名(超声○·深度··交叉○) 命中 {len(core)} 个测点")
    if core:
        print("         " + "、".join(core))
    return 0


if __name__ == "__main__":
    sys.exit(main())
