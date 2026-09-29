"""阶段 3：双雷达 Point-LIO + ROGMap + 隧道状态判定。

实时：  ros2 launch handheld_bringup s3_tunnel.launch.py csv_path:=$PWD/data/results/s3_tunnel_01.csv rviz:=true
回放：  ros2 launch handheld_bringup s3_tunnel.launch.py driver:=none use_sim_time:=true
        ros2 bag play data/bags/<bag> --clock
打点：  ros2 run handheld_bringup tunnel_event_marker.py   （p=放入箱体 r=移走箱体）

ROGMap 输入 /cloud_registered_full + /aft_mapped_to_init（camera_init 坐标系），
tunnel_status_node 统计 ROI 内 /rog_map/layer_type 并发布 /tunnel_status。
ROI 在 config/rog_map/tunnel_status.yaml，按洞口在 camera_init 下的实测位置修改（fixed 模式）。
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
    driver = LaunchConfiguration("driver").perform(context)
    use_sim_time = as_bool(LaunchConfiguration("use_sim_time").perform(context))
    csv_path = LaunchConfiguration("csv_path").perform(context)
    roi_mode = LaunchConfiguration("roi_mode").perform(context)

    lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "lidar_pointlio.launch.py")),
        launch_arguments={
            "driver": driver,
            "pointlio_config": "mid360_dual.yaml",
            "use_sim_time": str(use_sim_time).lower(),
            "rviz": LaunchConfiguration("rviz").perform(context),
            "rviz_config": "tunnel.rviz",
        }.items(),
    )
    rog_map = Node(
        package="handheld_bringup",
        executable="rog_map_node",
        name="rog_map_node",
        output="screen",
        parameters=[os.path.join(share, "config", "rog_map", "rog_map.yaml"), {"use_sim_time": use_sim_time}],
    )
    tunnel_params = {"use_sim_time": use_sim_time}
    if csv_path:
        tunnel_params["csv_path"] = os.path.abspath(os.path.expanduser(csv_path))
        os.makedirs(os.path.dirname(tunnel_params["csv_path"]), exist_ok=True)
    if roi_mode:
        tunnel_params["roi_mode"] = roi_mode
    tunnel = Node(
        package="handheld_bringup",
        executable="tunnel_status_node",
        name="tunnel_status_node",
        output="screen",
        parameters=[os.path.join(share, "config", "rog_map", "tunnel_status.yaml"), tunnel_params],
    )
    return [lio, rog_map, tunnel]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("driver", default_value="dual_merged", description="dual_merged | none（回放）"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("csv_path", default_value="", description="状态/延迟 CSV 输出路径，空 = 不写"),
        DeclareLaunchArgument("roi_mode", default_value="", description="fixed | body_relative，空 = 用 yaml"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
