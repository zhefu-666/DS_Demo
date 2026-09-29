"""离线回放：同一份原始 bag 分别跑 Point-LIO / FAST-LIO2 / small_point_lio（阶段 1、2、4 对照）。

每次只跑一个算法（三者 TF 与话题有重叠，且同时跑会互相抢 CPU，耗时数据失真）：
  ros2 launch handheld_bringup replay_lio.launch.py algo:=point_lio sensor:=dual
  ros2 bag play data/bags/<bag> --clock
推荐直接用 scripts/replay_eval.sh，它会自动启动算法、回放、录输出并转 TUM。

sensor 决定 IMU 话题与 Point-LIO 配置：
  single -> livox/imu                    + point_lio/mid360_single.yaml
  dual   -> livox/imu_192_168_1_135      + point_lio/mid360_dual.yaml
  hipnuc -> /imu/data                    + point_lio/mid360_hipnuc.yaml（仅 point_lio）
  front  -> 分路包，前雷达点云 + 前雷达 IMU  （仅 point_lio）
  back   -> 分路包，后雷达点云 + 后雷达 IMU  （仅 point_lio）
输出位姿话题：point_lio /aft_mapped_to_init；fast_lio /fast_lio/odometry；small_point_lio /small_point_lio/odometry
rviz:=true 使用 rviz/lio_compare.rviz，固定帧按算法自动设置（camera_init / fastlio_odom / odom）。
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from handheld_launch_utils import as_bool, back_lidar_blind_center  # noqa: E402  同目录模块

IMU_TOPICS = {
    "single": "livox/imu",
    "dual": "livox/imu_192_168_1_135",
    "hipnuc": "/imu/data",
    "front": "livox/imu_192_168_1_135",
    "back": "livox/imu_192_168_1_122",
}
POINTLIO_CONFIGS = {
    "single": "mid360_single.yaml",
    "dual": "mid360_dual.yaml",
    "hipnuc": "mid360_hipnuc.yaml",
    "front": "mid360_dual.yaml",
    "back": "mid360_dual.yaml",
}
LID_TOPICS = {
    "front": "livox/lidar_192_168_1_135",
    "back": "livox/lidar_192_168_1_122",
}
# 各算法的世界坐标系（RViz 固定帧）
WORLD_FRAMES = {
    "point_lio": "camera_init",
    "fast_lio": "fastlio_odom",
    "small_point_lio": "odom",
}


def rviz_node(share: str, algo: str) -> Node:
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(share, "rviz", "lio_compare.rviz"), "-f", WORLD_FRAMES[algo]],
        parameters=[{"use_sim_time": True}],
    )


def launch_setup(context):
    share = get_package_share_directory("handheld_bringup")
    algo = LaunchConfiguration("algo").perform(context)
    sensor = LaunchConfiguration("sensor").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)
    extra = [rviz_node(share, algo)] if as_bool(rviz) and algo in ("fast_lio", "small_point_lio") else []
    if sensor not in IMU_TOPICS:
        raise RuntimeError(f"sensor 必须是 {list(IMU_TOPICS)} 之一")
    imu_topic = IMU_TOPICS[sensor]
    lid_topic = LID_TOPICS.get(sensor, "livox/lidar")

    if algo == "point_lio":
        args = {
            "driver": "none",
            "pointlio_config": POINTLIO_CONFIGS[sensor],
            "lid_topic": lid_topic,
            "imu_topic": imu_topic,
            "use_sim_time": "true",
            "rviz": rviz,
            "rviz_config": "lio_compare.rviz",
            "save_pcd": LaunchConfiguration("save_pcd").perform(context),
        }
        if sensor == "back":
            args["blind_center"] = back_lidar_blind_center(share)
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "lidar_pointlio.launch.py")),
                launch_arguments=args.items(),
            )
        ]

    if sensor not in ("single", "dual"):
        raise RuntimeError(f"{algo} 只配置了 single/dual（Mid-360 内置 IMU），sensor={sensor} 仅支持 point_lio")

    if algo == "fast_lio":
        return [
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="laser_mapping_node",
                output="screen",
                parameters=[
                    os.path.join(share, "config", "fast_lio", "mid360.yaml"),
                    {"common.imu_topic": imu_topic, "common.lid_topic": "/livox/lidar", "use_sim_time": True},
                ],
                remappings=[
                    ("/Odometry", "/fast_lio/odometry"),
                    ("/path", "/fast_lio/path"),
                    ("/cloud_registered", "/fast_lio/cloud_registered"),
                    ("/cloud_registered_body", "/fast_lio/cloud_registered_body"),
                    ("/cloud_effected", "/fast_lio/cloud_effected"),
                    ("/Laser_map", "/fast_lio/Laser_map"),
                ],
            )
        ] + extra

    if algo == "small_point_lio":
        return [
            Node(
                package="small_point_lio",
                executable="small_point_lio_node",
                name="small_point_lio",
                output="screen",
                parameters=[
                    os.path.join(share, "config", "small_point_lio", "mid360.yaml"),
                    {"imu_topic": "/" + imu_topic.lstrip("/"), "lidar_topic": "/livox/lidar", "use_sim_time": True},
                ],
                remappings=[
                    ("/Odometry", "/small_point_lio/odometry"),
                    ("/cloud_registered", "/small_point_lio/cloud_registered"),
                ],
            ),
            # small_point_lio 用 TF 把雷达里程计换算到 base_link，这里令 base_link 与雷达重合
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="small_point_lio_base_to_lidar",
                arguments=["--frame-id", "base_link", "--child-frame-id", "livox_frame"],
                parameters=[{"use_sim_time": True}],
            ),
        ] + extra

    raise RuntimeError(f"algo 必须是 point_lio | fast_lio | small_point_lio，收到 '{algo}'")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("algo", default_value="point_lio", description="point_lio | fast_lio | small_point_lio"),
        DeclareLaunchArgument("sensor", default_value="dual", description="single | dual | hipnuc | front | back"),
        DeclareLaunchArgument("save_pcd", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
