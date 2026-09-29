#!/usr/bin/env bash
# 按阶段话题清单录原始包到 data/bags/<名称>。
#   scripts/record_stage.sh <清单> <名称> [额外 ros2 bag record 参数]
#   例：scripts/record_stage.sh s2_dual s2_loop_01
#       scripts/record_stage.sh s5_vio s5_corridor_01 -d 60
# 清单：src/handheld/handheld_bringup/config/record/<清单>.txt（每行一个话题，# 为注释）
# 存储：检测到 rosbag2_storage_mcap 时用 mcap，否则 sqlite3。
set -euo pipefail
source "$(dirname "$0")/common.sh"
LIST="${1:?用法: $0 <清单 s1_single|s2_dual|s2_split|s3_tunnel|s4_hipnuc|s5_vio|s6_reloc> <名称> [参数]}"
NAME="${2:?缺少包名称}"
shift 2
FILE="$WS_DIR/src/handheld/handheld_bringup/config/record/${LIST}.txt"
[[ -f "$FILE" ]] || { echo "清单不存在: $FILE" >&2; exit 1; }
OUT="$WS_DIR/data/bags/$NAME"
[[ -e "$OUT" ]] && { echo "已存在 $OUT，换个名称（原始包不覆盖）" >&2; exit 1; }
source_ws

mapfile -t TOPICS < <(sed -e 's/#.*//' -e 's/[[:space:]]//g' "$FILE" | grep -v '^$')
STORAGE=()
if ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1; then STORAGE=(-s mcap); fi

# 元数据：记录录包时的配置与 git 版本，方便事后复现
mkdir -p "$WS_DIR/data/bags"
META="$WS_DIR/data/bags/${NAME}.meta.txt"
{
  echo "name: $NAME"; echo "list: $LIST"; echo "start: $(date -Is)"; echo "host: $(hostname)"
  echo "git: $(git -C "$WS_DIR" rev-parse --short HEAD 2>/dev/null || echo none) $(git -C "$WS_DIR" status --porcelain 2>/dev/null | wc -l) dirty files"
  echo "topics:"; printf '  %s\n' "${TOPICS[@]}"
  echo "note: ${NOTE:-}"
} > "$META"

echo "录制 ${#TOPICS[@]} 个话题 -> $OUT （Ctrl-C 结束）"
exec ros2 bag record "${STORAGE[@]}" -o "$OUT" "$@" "${TOPICS[@]}"
