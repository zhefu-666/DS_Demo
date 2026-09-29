"""launch 文件共用的小工具（非 launch 文件，被同目录 launch 通过 sys.path 导入）。"""

import math
import os

import yaml


def as_bool(value: str) -> bool:
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def back_lidar_blind_center(share: str, front_center=(0.0, 0.20, 0.0)) -> str:
    """把前雷达坐标系下的 blind 球心换算到后雷达自身坐标系（仅后雷达对照用）。

    外参来自 config/driver/dual_merged.yaml 的 merge_extrinsic_back_to_front：
    [x, y, z, roll, pitch, yaw]，p_front = R * p_back + t，R = Rz * Ry * Rx
    => p_back = R^T * (p_front - t)
    """
    path = os.path.join(share, "config", "driver", "dual_merged.yaml")
    with open(path, "r", encoding="utf-8") as f:
        params = yaml.safe_load(f)["/**"]["ros__parameters"]
    x, y, z, roll, pitch, yaw = [float(v) for v in params["merge_extrinsic_back_to_front"]]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rot = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    d = [front_center[0] - x, front_center[1] - y, front_center[2] - z]
    back = [sum(rot[r][c] * d[r] for r in range(3)) for c in range(3)]  # R^T * d
    return ",".join(f"{v:.4f}" for v in back)
