#!/usr/bin/env bash
# 完整部署: 核心改动(scp) + ROS 包(WSL 构建/scp/Pi 构建) + tools/ 脚本 + 重启节点
# v2.9.21 (T-B1): 核心同步改**全量** *.cpp *.h —— 旧版手写清单漏 3 个 .cpp + 5 个头文件
#   (干净 Pi 链接失败 / 头文件静默错配 / 今后这些文件的改动上不了机)。
#   原则: Pi 上只增/覆盖, 绝不删除; 审计与复核都走全集合 md5。
set +u
K=/c/Users/老w/.ssh/id_ed25519; H=chj@192.168.123.86
CORE_SRC=/c/Users/老w/Documents/dsh/mechdog_navigation
CORE_DST=/home/chj/mdw_ws/src/mechdog_navigation

CORE_FILES=( "$CORE_SRC"/*.cpp "$CORE_SRC"/*.h )
N_LOCAL=${#CORE_FILES[@]}
norm() { awk -v src="$CORE_SRC/" '{n=$2; sub(/^\*/,"",n); sub(src,"",n); print $1" "n}'; }
LOCAL_MD5=$(md5sum "${CORE_FILES[@]}" | norm | sort)

echo "=== 0) 核心全量审计 (本地 $N_LOCAL 个 .cpp/.h vs Pi, md5) ==="
REMOTE_MD5=$(timeout 90 ssh -o ConnectTimeout=25 -o StrictHostKeyChecking=no -i $K $H \
  "cd $CORE_DST && md5sum *.cpp *.h 2>/dev/null" | norm | sort)
DIFF_CNT=0
while read -r m f; do
  r=$(echo "$REMOTE_MD5" | awk -v ff="$f" '$2==ff{print $1}')
  [ "$m" = "$r" ] || { echo "  DIFF  $f  local=$m pi=${r:-MISSING}"; DIFF_CNT=$((DIFF_CNT+1)); }
done <<< "$LOCAL_MD5"
echo "  差异: $DIFF_CNT / $N_LOCAL"
echo "  (Pi 侧多余文件, 无动作):"
echo "$REMOTE_MD5" | awk '{print $2}' | grep -vxF -f <(echo "$LOCAL_MD5" | awk '{print $2}') | sed 's/^/    /' | head -10

echo "=== 1) 传核心改动 (全量 $N_LOCAL 个) ==="
OK=0
for i in 1 2 3; do
  timeout 400 scp -q -o ConnectTimeout=40 -o StrictHostKeyChecking=no -i $K \
    "${CORE_FILES[@]}" "$H:$CORE_DST/" && { echo "  scp OK"; OK=1; break; }
  echo "  scp 重试 $i"; sleep 5
done
[ "$OK" = "1" ] || { echo "!! 核心 scp 失败, 中止"; exit 1; }

echo "=== 1b) 复核 (全量 md5 再比对) ==="
R2=$(timeout 90 ssh -o ConnectTimeout=25 -o StrictHostKeyChecking=no -i $K $H \
  "cd $CORE_DST && md5sum *.cpp *.h 2>/dev/null" | norm | sort)
MIS=0
while read -r m f; do
  r=$(echo "$R2" | awk -v ff="$f" '$2==ff{print $1}')
  [ "$m" = "$r" ] || { echo "  仍不一致: $f"; MIS=$((MIS+1)); }
done <<< "$LOCAL_MD5"
echo "  复核: $((N_LOCAL - MIS))/$N_LOCAL 一致"

echo "=== 2) ROS 包部署 (WSL 构建 + scp + Pi 构建) ==="
bash /c/Users/Public/deploy_roll.sh 2>&1 | grep -aE "finished|failed|error:|SCP_OK" | tail -4

echo "=== 2b) 推 tools/ 脚本到 /home/chj/ (T1: 此前不推 tools, 干净 Pi 缺脚本) ==="
TOOLS_SRC=/c/Users/老w/Documents/dsh/mechdog_navigation_ros/tools
for f in "$TOOLS_SRC"/*.sh "$TOOLS_SRC"/*.py; do
  base=$(basename "$f")
  timeout 120 scp -q -o ConnectTimeout=40 -o StrictHostKeyChecking=no -i $K "$f" "$H:/home/chj/$base" || echo "  scp FAIL $base"
done
timeout 60 ssh -o ConnectTimeout=30 -o StrictHostKeyChecking=no -i $K $H 'chmod +x /home/chj/*.sh' && echo "  tools/ scp done"

echo "=== 3) 重启节点并观察 ==="
timeout 240 ssh -o ConnectTimeout=30 -o StrictHostKeyChecking=no -i $K $H 'bash /home/chj/pi_tidy.sh >/dev/null 2>&1; sleep 15; grep -a "感知:" /home/chj/report_node.log | tail -2 | cut -c1-230' 2>/dev/null
