"""阶段 2 前置：双雷达外参标定录包。驱动分路发布（不合并），不启动 LIO。

  ros2 launch handheld_bringup s2_calib_record.launch.py
  scripts/record_stage.sh s2_split calib_$(date +%Y%m%d)
然后按 config/calibration/dual_mid360.yaml 顶部注释运行 dual_lidar_calibration。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory("handheld_bringup")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "driver.launch.py")),
            launch_arguments={"driver": "dual_split"}.items(),
        ),
    ])
