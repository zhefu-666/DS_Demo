#!/usr/bin/env python3
"""从 rosbag2 中导出位姿话题为 TUM 格式，供 evo 评估。

支持 nav_msgs/Odometry、geometry_msgs/PoseStamped、geometry_msgs/PoseWithCovarianceStamped。
时间戳默认取 header.stamp（传感器时间），可用 --use-bag-time 改为录包接收时间。

示例：
  ros2 run handheld_bringup bag_to_tum.py data/bags/s1_room_01 \
      --topic /aft_mapped_to_init --output data/results/s1_room_01_pointlio.tum
  # 一次导出多个话题（输出文件名按话题自动生成）
  ros2 run handheld_bringup bag_to_tum.py data/bags/s1_room_01 \
      --topic /aft_mapped_to_init /Odometry --output-dir data/results/s1_room_01
"""

import argparse
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

SUPPORTED_TYPES = {
    "nav_msgs/msg/Odometry",
    "geometry_msgs/msg/PoseStamped",
    "geometry_msgs/msg/PoseWithCovarianceStamped",
}


def detect_storage(bag_path: str) -> str:
    """根据目录中的文件扩展名判断存储插件。"""
    if os.path.isdir(bag_path):
        names = os.listdir(bag_path)
        if any(n.endswith(".mcap") for n in names):
            return "mcap"
        return "sqlite3"
    return "mcap" if bag_path.endswith(".mcap") else "sqlite3"


def extract_pose(msg, type_name: str):
    if type_name == "nav_msgs/msg/Odometry":
        return msg.header.stamp, msg.pose.pose
    if type_name == "geometry_msgs/msg/PoseWithCovarianceStamped":
        return msg.header.stamp, msg.pose.pose
    return msg.header.stamp, msg.pose


def topic_to_filename(topic: str) -> str:
    return topic.strip("/").replace("/", "_") + ".tum"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", help="rosbag2 目录或 .mcap 文件")
    parser.add_argument("--topic", nargs="+", required=True, help="位姿话题（可多个）")
    parser.add_argument("--output", help="单话题时的输出文件路径")
    parser.add_argument("--output-dir", help="多话题输出目录")
    parser.add_argument("--use-bag-time", action="store_true", help="使用录包接收时间而非 header.stamp")
    parser.add_argument("--storage", default="", help="强制指定存储插件 sqlite3/mcap")
    args = parser.parse_args()

    if len(args.topic) > 1 and not args.output_dir:
        parser.error("多个话题需要 --output-dir")
    if len(args.topic) == 1 and not (args.output or args.output_dir):
        parser.error("需要 --output 或 --output-dir")

    storage = args.storage or detect_storage(args.bag)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id=storage),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    outputs = {}
    for topic in args.topic:
        if topic not in type_map:
            print(f"[warn] 包中没有话题 {topic}，已跳过。现有话题: {sorted(type_map)}", file=sys.stderr)
            continue
        if type_map[topic] not in SUPPORTED_TYPES:
            print(f"[warn] {topic} 类型 {type_map[topic]} 不支持，已跳过", file=sys.stderr)
            continue
        if args.output_dir:
            os.makedirs(args.output_dir, exist_ok=True)
            path = os.path.join(args.output_dir, topic_to_filename(topic))
        else:
            os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
            path = args.output
        outputs[topic] = (open(path, "w", encoding="utf-8"), get_message(type_map[topic]), type_map[topic], path)

    if not outputs:
        print("[error] 没有可导出的话题", file=sys.stderr)
        return 1

    reader.set_filter(rosbag2_py.StorageFilter(topics=list(outputs)))
    counts = {t: 0 for t in outputs}
    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        fh, msg_type, type_name, _ = outputs[topic]
        msg = deserialize_message(data, msg_type)
        stamp, pose = extract_pose(msg, type_name)
        t = bag_time_ns * 1e-9 if args.use_bag_time else stamp.sec + stamp.nanosec * 1e-9
        p, q = pose.position, pose.orientation
        fh.write(f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")
        counts[topic] += 1

    for topic, (fh, _, _, path) in outputs.items():
        fh.close()
        print(f"{topic}: {counts[topic]} poses -> {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
