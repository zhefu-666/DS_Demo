#!/usr/bin/env python3
"""Summarize the common CSV contract, also usable by a future hardware observer."""
import argparse
import csv
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('csv')
args = parser.parse_args()
path = Path(args.csv)
with path.open() as f:
    rows = list(csv.DictReader(f))
front, back, pairs = map(int, Path(str(path) + '.counts').read_text().strip().split(','))
values = sorted(abs(float(r['delta_ms'])) for r in rows)
result = dict(front_received=front, back_received=back, pairs=pairs,
              front_unmatched=front-pairs, back_unmatched=back-pairs,
              pair_ratio=pairs/min(front, back) if min(front, back) else None,
              abs_delta_mean_ms=sum(values)/len(values) if values else None,
              abs_delta_p95_ms=values[max(0, __import__('math').ceil(.95*len(values))-1)] if values else None,
              abs_delta_max_ms=max(values) if values else None)
print(json.dumps(result, indent=2))
