#!/usr/bin/env bash
# 阶段 6 先验地图：把 Point-LIO 退出时写的 scans.pcd 清理后放到 data/maps/<名称>.pcd。
#   1) 建图：ros2 launch handheld_bringup s2_dual_pointlio.launch.py save_pcd:=true  （走完场地后 Ctrl-C）
#      或回放：ros2 launch handheld_bringup replay_lio.launch.py sensor:=dual save_pcd:=true + ros2 bag play --clock
#   2) scripts/save_map.sh <名称>
set -euo pipefail
source "$(dirname "$0")/common.sh"
NAME="${1:?用法: $0 <地图名称>}"
SRC="${SRC:-$WS_DIR/src/lio/point_lio/PCD/scans.pcd}"
DST="$WS_DIR/data/maps/${NAME}.pcd"
[[ -f "$SRC" ]] || { echo "未找到 $SRC（建图时需 save_pcd:=true，且正常 Ctrl-C 退出）" >&2; exit 1; }
[[ -e "$DST" ]] && { echo "已存在 $DST" >&2; exit 1; }
source_ws
mkdir -p "$WS_DIR/data/maps"
cp "$SRC" "$WS_DIR/data/maps/${NAME}_raw.pcd"
ros2 run icp_relocalization pcd_sanitize_tool "$WS_DIR/data/maps/${NAME}_raw.pcd" "$DST"
echo "地图：$DST （原始：data/maps/${NAME}_raw.pcd）"
