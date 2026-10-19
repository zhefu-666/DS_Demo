"""只启动 Livox 驱动（录原始包、双雷达标定录包、网络/PTP 检查）。

  ros2 launch handheld_bringup driver.launch.py driver:=dual_split     # 标定 / 仅前仅后录包
  ros2 launch handheld_bringup driver.launch.py driver:=dual_merged
  ros2 launch handheld_bringup driver.launch.py driver:=single_mid360
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def launch_setup(context):
    share = get_package_share_directory("handheld_bringup")
    driver = LaunchConfiguration("driver").perform(context)
    path = os.path.join(share, "config", "driver", f"{driver}.yaml")
    if not os.path.isfile(path):
        raise RuntimeError(f"驱动配置不存在: {path}")
    return [
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_driver_node",
            output="screen",
            parameters=[ParameterFile(path, allow_substs=True)],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("driver", default_value="dual_split",
                              description="single_mid360 | dual_merged | dual_split | dust_dual_split"),
        OpaqueFunction(function=launch_setup),
    ])
