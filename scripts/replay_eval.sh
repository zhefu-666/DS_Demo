#!/usr/bin/env bash
# 离线回放同一份原始包跑指定 LIO，录下输出位姿并导出 TUM，同时记录负载。
#   scripts/replay_eval.sh <bag> <algo> <sensor> [倍速]
#   algo:   point_lio | fast_lio | small_point_lio
#   sensor: single | dual | hipnuc | front | back   （见 replay_lio.launch.py）
#   例：scripts/replay_eval.sh data/bags/s1_room_01 fast_lio single
# 输出：data/results/<bag名>/<algo>_<sensor>.tum、<algo>_<sensor>_load.csv、<algo>_<sensor>_out/（输出包）
set -euo pipefail
source "$(dirname "$0")/common.sh"
BAG="$(realpath "${1:?用法: $0 <bag> <algo> <sensor> [倍速]}")"
ALGO="${2:?缺少 algo}"
SENSOR="${3:?缺少 sensor}"
RATE="${4:-1.0}"
source_ws

case "$ALGO" in
  point_lio)       POSE=/aft_mapped_to_init; PROC=component_container_mt ;;
  fast_lio)        POSE=/fast_lio/odometry;  PROC=fastlio_mapping ;;
  small_point_lio) POSE=/small_point_lio/odometry; PROC=small_point_lio_node ;;
  *) echo "未知 algo: $ALGO" >&2; exit 1 ;;
esac

RES="$WS_DIR/data/results/$(basename "$BAG")"
TAG="${ALGO}_${SENSOR}"
mkdir -p "$RES"
rm -rf "$RES/${TAG}_out"

PIDS=()
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT

ros2 launch handheld_bringup replay_lio.launch.py algo:="$ALGO" sensor:="$SENSOR" > "$RES/${TAG}_launch.log" 2>&1 &
PIDS+=($!)
sleep 5
ros2 bag record -o "$RES/${TAG}_out" --use-sim-time "$POSE" > /dev/null 2>&1 &
PIDS+=($!)
ros2 run handheld_bringup resource_monitor.py --output "$RES/${TAG}_load.csv" --match "$PROC" > "$RES/${TAG}_load.txt" 2>&1 &
MON=$!
sleep 2

echo "回放 $BAG （${RATE}x）..."
"$WS_DIR/scripts/play_raw.sh" "$BAG" -r "$RATE"
sleep 3
kill -INT "$MON" 2>/dev/null || true; wait "$MON" 2>/dev/null || true
cleanup
PIDS=()
trap - EXIT

ros2 run handheld_bringup bag_to_tum.py "$RES/${TAG}_out" --topic "$POSE" --output "$RES/${TAG}.tum"
cat "$RES/${TAG}_load.txt"
echo "完成：$RES/${TAG}.tum   日志 $RES/${TAG}_launch.log"
