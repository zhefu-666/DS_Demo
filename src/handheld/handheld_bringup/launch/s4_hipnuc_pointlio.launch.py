"""阶段 4：双雷达合并点云 + HIPNUC 替换主 IMU。

  ros2 launch handheld_bringup s4_hipnuc_pointlio.launch.py serial_port:=/dev/hipnuc
录包 s4_hipnuc 会同时保留 Mid-360 内置 IMU，离线可用
  replay_lio.launch.py sensor:=dual / sensor:=hipnuc
在同一份包上对比两种 IMU。
上车前必须完成：HIPNUC 到前雷达的外参（config/point_lio/mid360_hipnuc.yaml mapping.extrinsic_T/R）
以及时间偏移（config/hipnuc/hipnuc.yaml timestamp_offset_sec）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("handheld_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("save_pcd", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(
            package="ds_nav_imu",
            executable="hipnuc_imu_node",
            name="hipnuc_imu_node",
            output="screen",
            parameters=[
                os.path.join(share, "config", "hipnuc", "hipnuc.yaml"),
                {"serial_port": LaunchConfiguration("serial_port")},
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "lidar_pointlio.launch.py")),
            launch_arguments={
                "driver": "dual_merged",
                "pointlio_config": "mid360_hipnuc.yaml",
                "save_pcd": LaunchConfiguration("save_pcd"),
                "rviz": LaunchConfiguration("rviz"),
            }.items(),
        ),
    ])
