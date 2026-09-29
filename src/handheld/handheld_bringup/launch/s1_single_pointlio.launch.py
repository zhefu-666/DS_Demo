"""阶段 1：单 Mid-360 + Point-LIO 实时运行（驱动与 Point-LIO 同进程）。

  ros2 launch handheld_bringup s1_single_pointlio.launch.py rviz:=true
  另开终端录原始包：scripts/record_stage.sh s1_single s1_room_01
FAST-LIO2 / small_point_lio 对照一律离线回放同一份包（replay_lio.launch.py sensor:=single）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    core = os.path.join(get_package_share_directory("handheld_bringup"), "launch", "lidar_pointlio.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("save_pcd", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core),
            launch_arguments={
                "driver": "single_mid360",
                "pointlio_config": "mid360_single.yaml",
                "save_pcd": LaunchConfiguration("save_pcd"),
                "rviz": LaunchConfiguration("rviz"),
            }.items(),
        ),
    ])
