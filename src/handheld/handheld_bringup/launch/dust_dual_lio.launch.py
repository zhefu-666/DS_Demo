"""Dust 双雷达驱动内融合 + Point-LIO；新安装外参必须显式提供。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('handheld_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('merge_extrinsic', description='必填：.100 -> .136 的 x,y,z,roll,pitch,yaw，m/rad'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('save_pcd', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'lidar_pointlio.launch.py')),
            launch_arguments={
                'driver': 'dust_dual_merged',
                'pointlio_config': 'dust_dual.yaml',
                'merge_extrinsic': LaunchConfiguration('merge_extrinsic'),
                'rviz': LaunchConfiguration('rviz'),
                'save_pcd': LaunchConfiguration('save_pcd'),
            }.items()),
    ])
