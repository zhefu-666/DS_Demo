#!/usr/bin/env python3
"""Run real DDS/ApproximateTime integration scenarios, without hardware."""
import csv
import json
import os
from pathlib import Path
import subprocess
import sys

out = Path(sys.argv[1] if len(sys.argv) > 1 else '/tmp/dual_lidar_soft_sync')
out.mkdir(parents=True, exist_ok=True)
env = dict(os.environ, ROS_DOMAIN_ID='178', ROS_LOCALHOST_ONLY='1')
results = {}
for name, offset, jitter, tolerance, drop in [
        ('normal', 3., 1., 5., 0), ('reject', 5., 0., 1., 0),
        ('drop', 3., 1., 5., 5)]:
    path = out / (name + '.csv')
    params = dict(mock=True, duration_s=4., mock_offset_ms=offset,
                  mock_jitter_ms=jitter, max_delta_ms=tolerance,
                  mock_drop_every=drop, csv_path=str(path.resolve()))
    cmd = ['ros2', 'run', 'dual_lidar_soft_sync', 'soft_sync_demo', '--ros-args',
           '-r', '__ns:=/sync_compare/soft']
    for key, value in params.items():
        cmd += ['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}']
    with (out / (name + '.log')).open('w') as log:
        subprocess.run(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                       check=True, timeout=20)
    with path.open() as f:
        rows = list(csv.DictReader(f))
    front, back, pairs = map(int, Path(str(path)+'.counts').read_text().strip().split(','))
    assert front > 100 and back > 100, (name, front, back)
    assert pairs == len(rows)
    assert all(abs(float(r['delta_ms'])) <= tolerance + .001 for r in rows)
    assert len({r['front_stamp_ns'] for r in rows}) == pairs
    assert len({r['back_stamp_ns'] for r in rows}) == pairs
    if name == 'reject':
        assert pairs == 0, pairs
    else:
        assert pairs/min(front, back) > .90, (name, front, back, pairs)
        assert all(1.99 <= float(r['delta_ms']) <= 4.01 for r in rows)
    if name == 'drop':
        assert .75 < back/front < .85, (front, back)
    summary = json.loads(subprocess.check_output(
        [sys.executable, str(Path(__file__).with_name('summarize.py')), str(path)], text=True))
    results[name] = dict(parameters=params, metrics=summary, passed=True)
(out/'results.json').write_text(json.dumps(results, indent=2)+'\n')
print(json.dumps(results, indent=2))
