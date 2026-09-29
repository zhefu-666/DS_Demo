#!/usr/bin/env bash
# 只回放原始传感器话题（livox/*、/imu/data、/camera/*、/tunnel_event），带 --clock。
# 录包里的 /tf、/aft_mapped_to_init、/ov_msckf/* 等算法输出会被排除，否则会与回放时重新运行的算法冲突。
#   scripts/play_raw.sh <bag> [ros2 bag play 额外参数，例如 -r 0.5 --start-offset 10]
set -euo pipefail
source "$(dirname "$0")/common.sh"
BAG="${1:?用法: $0 <bag> [play 参数]}"
shift
source_ws
mapfile -t TOPICS < <(python3 - "$BAG" <<'PY'
import os, re, sys
import rosbag2_py
bag = sys.argv[1]
storage = "mcap" if (bag.endswith(".mcap") or (os.path.isdir(bag) and any(n.endswith(".mcap") for n in os.listdir(bag)))) else "sqlite3"
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
       rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
keep = re.compile(r"^/(livox/|imu/data$|camera/|tunnel_event$)")
for t in r.get_all_topics_and_types():
    if keep.match(t.name):
        print(t.name)
PY
)
[[ ${#TOPICS[@]} -gt 0 ]] || { echo "包中没有原始传感器话题" >&2; exit 1; }
echo "回放话题: ${TOPICS[*]}"
exec ros2 bag play "$BAG" --clock "$@" --topics "${TOPICS[@]}"
