"""Dust 100Hz driver + optional ApproximateTime observer; never changes hardware sync."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('handheld_bringup')
    soft = get_package_share_directory('dual_lidar_soft_sync')
    return LaunchDescription([
        DeclareLaunchArgument('soft_observer', default_value='false'),
        DeclareLaunchArgument('duration_s', default_value='60.0'),
        DeclareLaunchArgument('max_delta_ms', default_value='1.0'),
        DeclareLaunchArgument('csv_path', default_value='/tmp/dust_soft_pairs.csv'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'driver.launch.py')),
            launch_arguments={'driver': 'dust_dual_split'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(soft, 'launch', 'soft_sync.launch.py')),
            condition=IfCondition(LaunchConfiguration('soft_observer')),
            launch_arguments={
                'mock': 'false', 'front_topic': '/livox/lidar_192_168_25_100',
                'back_topic': '/livox/lidar_192_168_25_136',
                'duration_s': LaunchConfiguration('duration_s'),
                'max_delta_ms': LaunchConfiguration('max_delta_ms'),
                'csv_path': LaunchConfiguration('csv_path'),
            }.items()),
    ])
