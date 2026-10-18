#!/usr/bin/env python3
"""Read-only dual LiDAR header timestamp diagnostics; offline CSV needs no ROS."""
import argparse
import bisect
import csv
import json
import math
import time
from pathlib import Path


def stats(values):
    values = sorted(values)
    if not values:
        return None
    return dict(count=len(values), mean=sum(values)/len(values),
                p50=values[math.ceil(.50*len(values))-1],
                p95=values[math.ceil(.95*len(values))-1],
                p99=values[math.ceil(.99*len(values))-1],
                minimum=values[0], maximum=values[-1])


def analyze(rows, tolerance_ms=5., min_samples=100, coverage_required=.95):
    streams = {}
    stamps = {}
    for source in ('front', 'back'):
        data = [r for r in rows if r['source'] == source]
        stamps[source] = [int(r['stamp_ns']) for r in data]
        intervals = [(b-a)/1e6 for a, b in zip(stamps[source], stamps[source][1:])]
        arrivals = [int(r['receive_monotonic_ns']) for r in data]
        host_intervals = [(b-a)/1e6 for a, b in zip(arrivals, arrivals[1:])]
        positive = [v for v in intervals if v > 0]
        streams[source] = dict(received=len(data),
                               stamp_interval_ms=stats(intervals),
                               receive_interval_ms=stats(host_intervals),
                               non_increasing=sum(v <= 0 for v in intervals),
                               zero_stamp=sum(v == 0 for v in stamps[source]),
                               average_hz=1000/(sum(positive)/len(positive)) if positive else None)
    a, b = sorted(stamps['front']), sorted(stamps['back'])
    # Unfiltered nearest-neighbour deltas: back stamp - front stamp.
    # This measure permits re-use and must be read with the independent coverage metric.
    deltas = []
    for t in a:
        pos = bisect.bisect_left(b, t)
        candidates = b[max(0, pos-1):pos+1]
        if candidates:
            deltas.append((min(candidates, key=lambda s: abs(s-t))-t)/1e6)
    i = j = 0
    paired = []
    limit = tolerance_ms*1e6
    # Maximum-cardinality chronological one-to-one window matching; no interpolation.
    while i < len(a) and j < len(b):
        delta = b[j]-a[i]
        if abs(delta) <= limit:
            paired.append(delta/1e6)
            i += 1
            j += 1
        elif delta < 0:
            j += 1
        else:
            i += 1
    coverage = {source: len(paired)/len(stamps[source]) if stamps[source] else 0.
                for source in stamps}
    enough = all(len(stamps[s]) >= min_samples for s in stamps)
    valid = all(streams[s]['non_increasing'] == 0 and streams[s]['zero_stamp'] == 0
                for s in streams)
    good = enough and valid and all(c >= coverage_required for c in coverage.values())
    return dict(
        criteria=dict(tolerance_ms=tolerance_ms, min_samples=min_samples,
                      coverage_required=coverage_required),
        streams=streams, nearest_delta_ms=stats(deltas),
        nearest_abs_delta_ms=stats([abs(d) for d in deltas]),
        window_pairs=len(paired), coverage=coverage,
        window_abs_delta_ms=stats([abs(d) for d in paired]),
        verdict=('样本不足' if not enough else '时间戳接近且覆盖率达标' if good else '时间戳检查未达标'),
        limitation='仅判断消息头时间戳；周期数据可能跨帧误配，不能证明同次采集、逐点同步或物理硬同步。')


def collect(args, raw_path):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = Node('lidar_timestamp_check')
    rows = []
    with raw_path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['source', 'stamp_ns', 'receive_monotonic_ns'])
        writer.writeheader()

        def callback(source):
            def receive(msg):
                stamp = msg.header.stamp
                row = dict(source=source, stamp_ns=stamp.sec*10**9+stamp.nanosec,
                           receive_monotonic_ns=time.monotonic_ns())
                rows.append(row)
                writer.writerow(row)
            return receive

        message = get_message(args.message_type)
        subscriptions = [node.create_subscription(message, topic, callback(source), qos_profile_sensor_data)
                         for source, topic in [('front', args.front), ('back', args.back)]]
        start = last = time.monotonic()
        print(f'只读采集 {args.duration}s：{args.front} / {args.back}', flush=True)
        try:
            while rclpy.ok() and time.monotonic()-start < args.duration:
                rclpy.spin_once(node, timeout_sec=.1)
                if time.monotonic()-last >= 2:
                    counts = {s: sum(r['source'] == s for r in rows) for s in ('front', 'back')}
                    print(f'收到帧数 {counts}', flush=True)
                    f.flush()
                    last = time.monotonic()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--front', default='/livox/lidar_192_168_1_135')
    parser.add_argument('--back', default='/livox/lidar_192_168_1_122')
    parser.add_argument('--message-type', default='livox_ros_driver2/msg/CustomMsg')
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--tolerance-ms', type=float, default=5.)
    parser.add_argument('--min-samples', type=int, default=100)
    parser.add_argument('--coverage', type=float, default=.95)
    parser.add_argument('--from-csv', type=Path, help='离线重新分析本工具生成的 raw.csv，无需 ROS')
    parser.add_argument('--output', type=Path, default=Path('data/results/timestamp_'+time.strftime('%Y%m%d_%H%M%S')))
    args = parser.parse_args()
    if args.duration <= 0 or args.tolerance_ms <= 0 or args.min_samples < 2 or not 0 < args.coverage <= 1:
        parser.error('duration/tolerance > 0, min-samples >= 2, 0 < coverage <= 1')
    # Fail instead of silently overwriting previous experiments.
    args.output.mkdir(parents=True, exist_ok=False)
    if args.from_csv:
        with args.from_csv.open() as f:
            rows = list(csv.DictReader(f))
    else:
        rows = collect(args, args.output/'raw.csv')
    report = analyze(rows, args.tolerance_ms, args.min_samples, args.coverage)
    report['acquisition'] = {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}
    text = json.dumps(report, ensure_ascii=False, indent=2)
    (args.output/'report.json').write_text(text+'\n')
    print(text)
    print(f'结果：{args.output.resolve()}')


if __name__ == '__main__':
    main()
