"""阶段 6：出口重定位（Point-LIO 双雷达 + small_gicp 对齐预建地图）。

  ros2 launch handheld_bringup s6_relocalization.launch.py \
      map_file:=$PWD/data/maps/field.pcd init_x:=0.0 init_y:=0.0 init_yaw:=0.0 \
      results_csv:=$PWD/data/results/s6_exit_01.csv
回放：加 driver:=none use_sim_time:=true，再 ros2 bag play <bag> --clock
重新搜索：ros2 service call /gicp_recall std_srvs/srv/Trigger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def as_bool(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes", "on")


def launch_setup(context):
    share = get_package_share_directory("handheld_bringup")
    map_file = os.path.abspath(os.path.expanduser(LaunchConfiguration("map_file").perform(context)))
    if not os.path.isfile(map_file):
        raise RuntimeError(f"先验地图不存在: {map_file}（map_file 需指向 .pcd，建议放 data/maps/）")
    use_sim_time = as_bool(LaunchConfiguration("use_sim_time").perform(context))
    mode = LaunchConfiguration("mode").perform(context)
    init = [float(LaunchConfiguration(k).perform(context)) for k in ("init_x", "init_y", "init_yaw")]
    results_csv = LaunchConfiguration("results_csv").perform(context)

    overrides = {
        "use_sim_time": use_sim_time,
        "target_pcd_file": map_file,
        "mode": mode,
        "initial_pose": [init[0], init[1], 0.0, 0.0, 0.0, init[2]],
    }
    if results_csv:
        results_csv = os.path.abspath(os.path.expanduser(results_csv))
        os.makedirs(os.path.dirname(results_csv), exist_ok=True)
        overrides["results.path"] = results_csv

    lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "lidar_pointlio.launch.py")),
        launch_arguments={
            "driver": LaunchConfiguration("driver").perform(context),
            "pointlio_config": "mid360_dual.yaml",
            "use_sim_time": str(use_sim_time).lower(),
            "rviz": LaunchConfiguration("rviz").perform(context),
            "rviz_config": "reloc.rviz",
        }.items(),
    )
    gicp = Node(
        package="icp_relocalization",
        executable="gicp_node",
        name="gicp_relocalization_node",
        output="screen",
        parameters=[os.path.join(share, "config", "gicp", "relocalization.yaml"), overrides],
    )
    return [lio, gicp]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_file", description="预建地图 .pcd（绝对或相对当前目录）"),
        DeclareLaunchArgument("mode", default_value="initial_guess", description="initial_guess | multi_guess"),
        DeclareLaunchArgument("init_x", default_value="0.0"),
        DeclareLaunchArgument("init_y", default_value="0.0"),
        DeclareLaunchArgument("init_yaw", default_value="0.0", description="rad"),
        DeclareLaunchArgument("results_csv", default_value=""),
        DeclareLaunchArgument("driver", default_value="dual_merged", description="dual_merged | none（回放）"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
