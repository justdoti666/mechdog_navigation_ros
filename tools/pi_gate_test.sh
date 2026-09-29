#!/usr/bin/env bash
# v2 (2026-09-29, T6) 深度质量守门 —— 真机注入回归（真注入 + 真断言 + PASS/FAIL）
#   复现实机事故: 深度输出全 0 帧 ⇒ ①每轮被守门拦下(valid=0.0% / 0 px, 本轮不注入地形)
#   ②连续坏帧按 depth_bad_streak_n 升级"降级"; ③无帧启动 ⇒ 启动期降级(N9)。
#   断言依据(当前实现, 逐行核对过): safety_node.cpp:907-909(守门行) / :716-734(降级行) / :853-861(状态文案)
#   注入手段: pi_replay.py 单帧模式(10Hz) —— 不依赖相机硬件, 喂的是造帧。
# 用法:
#   bash pi_gate_test.sh                                             # Pi 全流程(≈1 分钟)
#   PI_HOME=/tmp/x MWD_WS=/tmp/mdw INJECT_SECS=12 bash pi_gate_test.sh   # 离线(WSL)自测用
set +u
PI_HOME="${PI_HOME:-/home/chj}"
MWD_WS="${MWD_WS:-/home/chj/mdw_ws}"
INJECT_SECS="${INJECT_SECS:-20}"
export ROS_DOMAIN_ID=42
for _d in /opt/ros/*/setup.bash; do [ -f "$_d" ] && { source "$_d"; break; }; done
[ -f "$MWD_WS/install/setup.bash" ] && source "$MWD_WS/install/setup.bash"
LOG="$PI_HOME/report_node.log"
RLOG="$PI_HOME/gate_replay.log"

FAILS=()
say() { echo "$@"; }
ok()  { say "  ✓ $1"; }
bad() { FAILS+=("$1"); say "  ✗ $1"; }

echo "=== 0) 清理: 停相机/旧回放/旧节点, 重置日志 (PI_HOME=$PI_HOME) ==="
pkill -9 -f "[a]stra_camera_node" 2>/dev/null
pkill -9 -f "[p]i_replay" 2>/dev/null
pkill -9 -f "[s]afety_node" 2>/dev/null
sleep 3
: > "$LOG"; : > "$RLOG"

setsid ros2 run mechdog_navigation_ros safety_node --ros-args \
  -p use_simulated:=false -p depth_source:=topic -p camera_height_m:=0.75 \
  -p cloud_z:=0.0 -p ground_fit_method:=cell \
  -p cloud_pitch_rad:=0.0 -p cloud_roll_rad:=0.0 -p enable_pointcloud:=true \
  > "$LOG" 2>&1 < /dev/null &
sleep 10

echo "=== A) 基线断言: 无帧启动 ⇒ 启动期降级必须出现 (N9) ==="
if pgrep -f "[s]afety_node" >/dev/null; then ok "节点存活"; else bad "节点未启动"; fi
N_START_DEG=$(grep -ac "深度降级(启动期)" "$LOG")
if [ "$N_START_DEG" -ge 1 ]; then ok "启动期降级 ×$N_START_DEG"; else bad "启动期降级未出现(期望≥1, 实得 $N_START_DEG)"; fi
grep -a "深度降级(启动期)" "$LOG" | tail -1 | cut -c1-170
N_GATE0=$(grep -ac "深度质量未就绪" "$LOG")
say "  (基线守门行=$N_GATE0 — 无帧≠坏帧, 预期 0)"

echo "=== B) 造全 0 帧 (640x480 16UC1) + 真注入 ${INJECT_SECS}s @10Hz ==="
python3 - "$PI_HOME/zero_depth.raw" <<'PY'
import sys, numpy as np
open(sys.argv[1], 'wb').write(np.zeros((480, 640), dtype=np.uint16).tobytes())
print('zero frame written 614400 bytes')
PY
SZ=$(stat -c%s "$PI_HOME/zero_depth.raw" 2>/dev/null)
if [ "$SZ" = "614400" ]; then ok "zero_depth.raw = 614400 B"; else bad "zero_depth.raw 大小异常: $SZ"; fi

setsid python3 "$PI_HOME/pi_replay.py" "$PI_HOME/zero_depth.raw" "$INJECT_SECS" > "$RLOG" 2>&1 < /dev/null &
sleep $((INJECT_SECS + 4))

echo "=== C) 注入断言: 坏帧到达节点 ⇒ 守门拦下 ⇒ 降级 ==="
FRAMES=$(grep -a "REPLAY_DONE" "$RLOG" | tail -1 | awk -F'frames=' '{print $2}' | tr -d ' \r')
if [ -n "$FRAMES" ] && [ "$FRAMES" -ge $((INJECT_SECS * 8)) ]; then
  ok "回放器确实发出 $FRAMES 帧 (≥$((INJECT_SECS * 8)))"
else
  bad "回放未成功(REPLAY_DONE frames=$FRAMES) — 注入失败, 本次不算回归!"
fi
N_GATE=$(grep -ac "深度质量未就绪" "$LOG")
if [ "$N_GATE" -ge 3 ]; then ok "守门拦下 ×$N_GATE (2s 节流下限)"; else bad "守门行不足: $N_GATE (期望≥3)"; fi
N_GATE_Z=$(grep -a "深度质量未就绪" "$LOG" | grep -ac "valid=0.0% / 0 px")
if [ "$N_GATE_Z" -ge 1 ]; then ok "全 0 帧特征 'valid=0.0% / 0 px' ×$N_GATE_Z"; else bad "未见 valid=0.0% / 0 px — 未证明坏帧到达节点"; fi
N_DEG=$(grep -acE "深度降级|DEPTH DEGRADED" "$LOG")
if [ "$N_DEG" -ge 1 ]; then ok "降级路径出现 ×$N_DEG"; else bad "无降级行"; fi
echo "  ── 证据样本 ──"
grep -a "深度质量未就绪" "$LOG" | tail -2 | cut -c1-170

echo "=== D) 收尾 ==="
pkill -9 -f "[p]i_replay" 2>/dev/null
say "  回放已停; 测试节点仍在运行(带测试参数)。现场恢复: bash $PI_HOME/pi_bringup_all.sh (相机→节点→窗口)"
say "  (相机不在位时恢复先跳过; 本测试不依赖相机硬件)"

echo
if [ ${#FAILS[@]} -eq 0 ]; then
  echo "RESULT: PASS  (守门 ${N_GATE} 行 / 启动降级 ${N_START_DEG} / 全0特征 ${N_GATE_Z} / 回放 ${FRAMES} 帧)"
  exit 0
else
  echo "RESULT: FAIL  (${#FAILS[@]} 项未过) — 这是真问题, 不要当作'跳过'"
  for f in "${FAILS[@]}"; do echo "   - $f"; done
  exit 1
fi
