"""阶段 2：双 Mid-360 + Point-LIO 实时运行。

mode:
  merged      驱动内部合并（主方案），点云 livox/lidar，IMU 用前雷达 livox/imu_192_168_1_135
  front_only  驱动分路发布，只喂前雷达点云 + 前雷达 IMU
  back_only   驱动分路发布，只喂后雷达点云 + 后雷达 IMU（blind 球心自动换算到后雷达坐标系）

注意：merged 模式下任一雷达掉线，合并输出会停（驱动日志 "waiting for front/back lidar data"），
Point-LIO 随之无点云输入——这正是遮挡/掉线测试要记录的现象。
推荐做法：实时只跑 merged 并录 s2_dual；前/后单路对照用 s2_split 包离线回放
（replay_lio.launch.py sensor:=front/back），保证同一份数据。
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from handheld_launch_utils import back_lidar_blind_center  # noqa: E402  同目录模块

MODES = {
    "merged": {"driver": "dual_merged"},
    "front_only": {
        "driver": "dual_split",
        "lid_topic": "livox/lidar_192_168_1_135",
        "imu_topic": "livox/imu_192_168_1_135",
    },
    "back_only": {
        "driver": "dual_split",
        "lid_topic": "livox/lidar_192_168_1_122",
        "imu_topic": "livox/imu_192_168_1_122",
    },
}


def launch_setup(context):
    share = get_package_share_directory("handheld_bringup")
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in MODES:
        raise RuntimeError(f"mode 必须是 {list(MODES)} 之一，收到 '{mode}'")
    args = dict(MODES[mode])
    args.update({
        "pointlio_config": "mid360_dual.yaml",
        "save_pcd": LaunchConfiguration("save_pcd").perform(context),
        "rviz": LaunchConfiguration("rviz").perform(context),
    })
    if mode == "back_only":
        args["blind_center"] = back_lidar_blind_center(share)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "lidar_pointlio.launch.py")),
            launch_arguments=args.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="merged", description="merged | front_only | back_only"),
        DeclareLaunchArgument("save_pcd", default_value="false", description="阶段 6 建先验地图时设 true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
