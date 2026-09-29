#!/usr/bin/env python3
"""NUC 运行负载记录（只依赖 /proc 与 /sys，无需 psutil）。

按进程名/命令行关键字统计 CPU%（以单核 100% 计）与 RSS，同时记录整机 CPU、内存和最高温度。

示例：
  ros2 run handheld_bringup resource_monitor.py --output data/results/s2_load.csv \
      --match component_container pointlio_mapping fastlio_mapping rog_map_node gicp_node run_subscribe_msckf
"""

import argparse
import csv
import glob
import os
import signal
import sys
import time

CLK_TCK = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
PAGE_SIZE = os.sysconf("SC_PAGE_SIZE")
NUM_CPUS = os.cpu_count() or 1


def read_system_cpu():
    with open("/proc/stat", "r", encoding="utf-8") as f:
        fields = [int(v) for v in f.readline().split()[1:]]
    idle = fields[3] + fields[4]  # idle + iowait
    return sum(fields), idle


def read_meminfo():
    info = {}
    with open("/proc/meminfo", "r", encoding="utf-8") as f:
        for line in f:
            key, value = line.split(":", 1)
            info[key] = int(value.split()[0])  # kB
    return info["MemTotal"], info.get("MemAvailable", info["MemFree"])


def read_max_temp():
    temps = []
    for path in glob.glob("/sys/class/thermal/thermal_zone*/temp"):
        try:
            with open(path, "r", encoding="utf-8") as f:
                temps.append(int(f.read().strip()) / 1000.0)
        except (OSError, ValueError):
            continue
    return max(temps) if temps else float("nan")


def read_proc(pid):
    """返回 (cmdline, utime+stime ticks, rss bytes)；进程已退出时返回 None。"""
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            cmdline = f.read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
        with open(f"/proc/{pid}/stat", "r", encoding="utf-8") as f:
            stat = f.read()
        # comm 字段可能包含空格，从最后一个 ')' 之后解析
        rest = stat[stat.rfind(")") + 2:].split()
        ticks = int(rest[11]) + int(rest[12])
        rss = int(rest[21]) * PAGE_SIZE
        if not cmdline:
            cmdline = stat[stat.find("(") + 1:stat.rfind(")")]
        return cmdline, ticks, rss
    except (OSError, IndexError, ValueError):
        return None


def match_processes(patterns):
    """关键字 -> {pid: (ticks, rss)}；一个进程只归到第一个匹配的关键字。"""
    result = {p: {} for p in patterns}
    self_pid = os.getpid()
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) == self_pid:
            continue
        info = read_proc(entry)
        if info is None:
            continue
        cmdline, ticks, rss = info
        for pattern in patterns:
            if pattern in cmdline:
                result[pattern][int(entry)] = (ticks, rss)
                break
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", required=True, help="CSV 输出路径")
    parser.add_argument("--match", nargs="+", required=True, help="进程命令行关键字")
    parser.add_argument("--period", type=float, default=1.0, help="采样周期 s")
    parser.add_argument("--duration", type=float, default=0.0, help="总时长 s，0 表示直到 Ctrl-C")
    args = parser.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    header = ["wall_time", "sys_cpu_pct", "mem_used_mb", "mem_total_mb", "max_temp_c"]
    for p in args.match:
        header += [f"{p}_cpu_pct", f"{p}_rss_mb", f"{p}_nproc"]

    running = True

    def stop(_sig, _frm):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    prev_sys = read_system_cpu()
    prev_procs = match_processes(args.match)
    prev_t = time.monotonic()
    start = prev_t
    summary = {p: [] for p in args.match}

    with open(args.output, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        while running:
            time.sleep(args.period)
            now = time.monotonic()
            dt = now - prev_t
            cur_sys = read_system_cpu()
            total_d = cur_sys[0] - prev_sys[0]
            idle_d = cur_sys[1] - prev_sys[1]
            sys_cpu = 100.0 * (1.0 - idle_d / total_d) if total_d > 0 else 0.0
            mem_total, mem_avail = read_meminfo()
            cur_procs = match_processes(args.match)

            row = [f"{time.time():.3f}", f"{sys_cpu:.1f}", f"{(mem_total - mem_avail) / 1024:.0f}",
                   f"{mem_total / 1024:.0f}", f"{read_max_temp():.1f}"]
            for p in args.match:
                cpu_ticks = 0
                rss = 0
                for pid, (ticks, r) in cur_procs[p].items():
                    prev = prev_procs[p].get(pid)
                    if prev is not None:
                        cpu_ticks += ticks - prev[0]
                    rss += r
                cpu_pct = 100.0 * cpu_ticks / CLK_TCK / dt if dt > 0 else 0.0
                row += [f"{cpu_pct:.1f}", f"{rss / 1048576:.1f}", len(cur_procs[p])]
                if cur_procs[p]:
                    summary[p].append((cpu_pct, rss / 1048576))
            writer.writerow(row)
            f.flush()

            prev_sys, prev_procs, prev_t = cur_sys, cur_procs, now
            if args.duration > 0 and now - start >= args.duration:
                break

    print(f"已写入 {args.output}（CPU% 以单核 100% 计，本机 {NUM_CPUS} 核）")
    for p, samples in summary.items():
        if samples:
            cpus = [s[0] for s in samples]
            rss = [s[1] for s in samples]
            print(f"  {p}: CPU 平均 {sum(cpus) / len(cpus):.1f}% 峰值 {max(cpus):.1f}%  RSS 峰值 {max(rss):.0f} MB")
        else:
            print(f"  {p}: 未匹配到进程")
    return 0


if __name__ == "__main__":
    sys.exit(main())
