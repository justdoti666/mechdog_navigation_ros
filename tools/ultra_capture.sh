#!/usr/bin/env bash
# ultra_capture.sh — 超声真机取证工装 (U-6, 2026-10-05)
#
# 前提: 目标机上 ultrasonic_node 已在运行 (本工装只抓证据, 不启停节点).
# 产物(输出目录):
#   META.txt          时间(本地+unix)/hostname/内核/节点进程/ROS_DISTRO/域/工作区
#   frame_01.txt ...  逐帧字段快照 (rclpy 直捕, 复用同目录 ultra_frame_capture.py)
#   node_stdout.txt   节点日志 (auto=journalctl 找 ultrasonic 行; 或传文件路径复制)
#   MANIFEST.md5      产物 md5 清单 (+ MD5_VERIFY.txt 校验输出)
# 新鲜度约定: scp 默认不带 -p ⇒ 远端 mtime 会变, 复核只认 md5 清单与 Pi 端记录时间.
# 退出码: 0=全部帧抓取成功; 1=有帧失败(产物仍保留, 见各 frame_*.txt); 2=用法错.
# 用法: bash ultra_capture.sh <输出目录> [帧数=5] [日志来源 auto|<路径>]
# 例:   bash ultra_capture.sh ~/ev/20261005_u1 5 auto
set -uo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
OUT="${1:?用法: ultra_capture.sh <输出目录> [帧数=5] [日志来源 auto|<路径>]}"
N="${2:-5}"
LOG_SRC="${3:-auto}"
mkdir -p "$OUT"

echo "=== [1] META ==="
{
  echo "# ultra_capture META (U-6 工装, 2026-10-05)"
  echo "time_local: $(date '+%F %T %z')"
  echo "time_unix:  $(date +%s)"
  echo "hostname:   $(hostname)"
  echo "kernel:     $(uname -r)"
  echo "ros_distro: ${ROS_DISTRO:-<未 source ROS>}"
  echo "ros_domain: ${ROS_DOMAIN_ID:-<默认 0>}"
  echo "workspace:  ${AMENT_PREFIX_PATH:-<空>}"
  echo "node_proc:"
  pgrep -af ultrasonic_node 2>/dev/null || echo "  (未发现 ultrasonic_node 进程 —— 确认节点已启动?)"
} > "$OUT/META.txt" 2>&1
cat "$OUT/META.txt"

echo "=== [2] 抓 $N 帧 ==="
FAILN=0
i=1
while [ "$i" -le "$N" ]; do
  f=$(printf "%s/frame_%02d.txt" "$OUT" "$i")
  timeout 15 python3 "$SCRIPT_DIR/ultra_frame_capture.py" > "$f" 2>&1
  r=$?
  echo "  frame_$i rc=$r -> $f"
  [ "$r" -ne 0 ] && FAILN=$((FAILN+1))
  i=$((i+1))
done

echo "=== [3] 节点日志 ($LOG_SRC) ==="
if [ "$LOG_SRC" = "auto" ]; then
  if command -v journalctl >/dev/null 2>&1; then
    journalctl -n 300 --no-pager 2>/dev/null | grep -iE "ultrasonic" > "$OUT/node_stdout.txt" || true
  fi
  [ -s "$OUT/node_stdout.txt" ] || echo "(auto 模式未捞到: 若节点前台手动运行, 请把其 stdout 文件路径作为第 3 参传入)" > "$OUT/node_stdout.txt"
else
  cp -f "$LOG_SRC" "$OUT/node_stdout.txt" 2>/dev/null || echo "(复制失败: $LOG_SRC)" > "$OUT/node_stdout.txt"
fi
tail -5 "$OUT/node_stdout.txt" 2>/dev/null

echo "=== [4] md5 清单 ==="
( cd "$OUT" && md5sum META.txt frame_*.txt node_stdout.txt > MANIFEST.md5 2>/dev/null )
( cd "$OUT" && md5sum -c MANIFEST.md5 ) 2>&1 | tee "$OUT/MD5_VERIFY.txt"

if [ "$FAILN" -gt 0 ]; then
  echo "DONE(有失败): $OUT —— $FAILN/$N 帧抓取失败 (原因见各 frame_*.txt)"
  exit 1
fi
echo "DONE: $OUT (新鲜度复核用 MANIFEST.md5 + META.txt 时间; 勿依赖 scp 后 mtime)"
