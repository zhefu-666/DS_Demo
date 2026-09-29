#!/usr/bin/env bash
# evo 轨迹评估。两种用法：
#  1) 两条轨迹互比（例如 Point-LIO vs FAST-LIO，或 OpenVINS vs 雷达参考）：
#       scripts/eval_evo.sh pair <参考.tum> <待测.tum> [输出目录]
#     做 SE(3) 对齐后的 APE/RPE（OpenVINS 与雷达坐标系不同，必须对齐；单目需加 --correct_scale 手动跑）
#  2) 重复路线 + 实测停止点：
#       scripts/eval_evo.sh stops <轨迹.tum> <stops.csv> [输出目录]
#     stops.csv 每行：t_start,t_end,x,y（录包时间 s，停留区间与卷尺/全站仪实测的停止点坐标，同一坐标系）
#     输出每个停止点的平均位置误差。
set -euo pipefail
source "$(dirname "$0")/common.sh"
MODE="${1:?用法见脚本头部}"
command -v evo_ape >/dev/null || { echo "未安装 evo：python3 -m pip install --user evo==1.31.1" >&2; exit 1; }

case "$MODE" in
  pair)
    REF="${2:?缺少参考轨迹}"; EST="${3:?缺少待测轨迹}"
    OUT="${4:-$(dirname "$EST")/evo_$(basename "${EST%.tum}")_vs_$(basename "${REF%.tum}")}"
    mkdir -p "$OUT"
    evo_ape tum "$REF" "$EST" -a --t_max_diff 0.02 --save_results "$OUT/ape.zip" | tee "$OUT/ape.txt"
    evo_rpe tum "$REF" "$EST" -a --t_max_diff 0.02 --delta 1 --delta_unit m --all_pairs \
      --save_results "$OUT/rpe_1m.zip" | tee "$OUT/rpe_1m.txt"
    evo_traj tum "$REF" "$EST" --ref "$REF" -a --save_plot "$OUT/traj.pdf" --plot_mode xy >/dev/null 2>&1 || \
      evo_traj tum "$REF" "$EST" --ref "$REF" -a | tee "$OUT/traj.txt"
    echo "结果：$OUT"
    ;;
  stops)
    TRAJ="${2:?缺少轨迹}"; STOPS="${3:?缺少 stops.csv}"
    OUT="${4:-$(dirname "$TRAJ")}"
    mkdir -p "$OUT"
    python3 - "$TRAJ" "$STOPS" "$OUT/$(basename "${TRAJ%.tum}")_stops.csv" <<'PY'
import csv, math, sys
traj, stops, out = sys.argv[1:4]
poses = []
with open(traj) as f:
    for line in f:
        v = line.split()
        if len(v) >= 4 and not line.startswith("#"):
            poses.append((float(v[0]), float(v[1]), float(v[2])))
rows = []
with open(stops) as f:
    for i, r in enumerate(csv.reader(f)):
        if not r or r[0].strip().startswith("#") or r[0].strip() == "t_start":
            continue
        t0, t1, x, y = map(float, r[:4])
        pts = [p for p in poses if t0 <= p[0] <= t1]
        if not pts:
            rows.append((i, t0, t1, x, y, float("nan"), float("nan"), float("nan"), 0)); continue
        mx = sum(p[1] for p in pts) / len(pts); my = sum(p[2] for p in pts) / len(pts)
        rows.append((i, t0, t1, x, y, mx, my, math.hypot(mx - x, my - y), len(pts)))
with open(out, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["idx", "t_start", "t_end", "gt_x", "gt_y", "est_x", "est_y", "err_xy", "n"])
    w.writerows(rows)
errs = [r[7] for r in rows if r[8] > 0]
for r in rows:
    print(f"stop {r[0]}: gt=({r[3]:.3f},{r[4]:.3f}) est=({r[5]:.3f},{r[6]:.3f}) err={r[7]:.3f} m n={r[8]}")
if errs:
    print(f"平均 {sum(errs)/len(errs):.3f} m  最大 {max(errs):.3f} m  （{len(errs)}/{len(rows)} 个停止点有数据）")
print(f"-> {out}")
PY
    ;;
  *) echo "未知模式 $MODE（pair | stops）" >&2; exit 1 ;;
esac
