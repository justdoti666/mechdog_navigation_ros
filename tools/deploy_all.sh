#!/usr/bin/env bash
# 完整部署: 核心改动(scp) + ROS 包(WSL 构建/scp/Pi 构建) + 重启节点
set +u
K=/c/Users/老w/.ssh/id_ed25519; H=chj@192.168.123.86
CORE_SRC=/c/Users/老w/Documents/dsh/mechdog_navigation
CORE_DST=/home/chj/mdw_ws/src/mechdog_navigation

echo "=== 0) 核心改动清单 (本地 vs Pi, md5) ==="
for f in heightmap_2d5.h heightmap_2d5.cpp ground_segmentation.h ground_segmentation.cpp \
         sensor_fusion.h sensor_fusion.cpp config.h; do
  L=$(md5sum "$CORE_SRC/$f" | cut -c1-8)
  R=$(timeout 60 ssh -o ConnectTimeout=25 -o StrictHostKeyChecking=no -i $K $H "md5sum $CORE_DST/$f 2>/dev/null | cut -c1-8")
  [ "$L" = "$R" ] && echo "  same  $f ($L)" || echo "  DIFF  $f  local=$L pi=$R"
done

echo "=== 1) 传核心改动 ==="
for f in heightmap_2d5.h heightmap_2d5.cpp ground_segmentation.h ground_segmentation.cpp \
         sensor_fusion.h sensor_fusion.cpp config.h; do
  timeout 120 scp -q -o ConnectTimeout=40 -o StrictHostKeyChecking=no -i $K "$CORE_SRC/$f" "$H:$CORE_DST/$f" || echo "  scp FAIL $f"
done
echo "  scp done"

echo "=== 2) ROS 包部署 (WSL 构建 + scp + Pi 构建) ==="
bash /c/Users/Public/deploy_roll.sh 2>&1 | grep -aE "finished|failed|error:|SCP_OK" | tail -4

echo "=== 3) 重启节点并观察 ==="
timeout 240 ssh -o ConnectTimeout=30 -o StrictHostKeyChecking=no -i $K $H 'bash /home/chj/pi_tidy.sh >/dev/null 2>&1; sleep 15; grep -a "感知:" /home/chj/report_node.log | tail -2 | cut -c1-230' 2>/dev/null
