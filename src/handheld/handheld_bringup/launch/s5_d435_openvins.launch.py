"""阶段 5：D435 红外双目 + HIPNUC + OpenVINS，与雷达里程计独立运行、分别评估（不做位姿平均）。

采集（推荐）：视觉与雷达同时录原始包，OpenVINS 离线回放，雷达结果作为参考轨迹
  ros2 launch handheld_bringup s5_d435_openvins.launch.py openvins:=false lidar:=true
  scripts/record_stage.sh s5_vio s5_corridor_01
实时跑 OpenVINS（观察效果/测负载）：
  ros2 launch handheld_bringup s5_d435_openvins.launch.py
回放：
  ros2 launch handheld_bringup s5_d435_openvins.launch.py camera:=false imu:=false use_sim_time:=true
  ros2 bag play data/bags/<bag> --clock
输出：/ov_msckf/poseimu（global -> imu），与 /aft_mapped_to_init 分别转 TUM 后用 evo 对齐比较。
显示：rviz:=true 打开 rviz/vio.rviz（固定帧 global，含特征跟踪图 trackhist）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def as_bool(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes", "on")


def launch_setup(context):
    share = get_package_share_directory("handheld_bringup")
    use_sim_time = as_bool(LaunchConfiguration("use_sim_time").perform(context))
    actions = []

    if as_bool(LaunchConfiguration("camera").perform(context)):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")),
            launch_arguments={
                "camera_name": "camera",
                "camera_namespace": "camera",
                "config_file": os.path.join(share, "config", "realsense", "d435_ir.yaml"),
                "enable_color": "false",
                "enable_depth": "false",
                "enable_infra1": "true",
                "enable_infra2": "true",
                "depth_module.infra_profile": LaunchConfiguration("infra_profile").perform(context),
                "enable_sync": "true",
                "enable_gyro": "false",
                "enable_accel": "false",
                "pointcloud.enable": "false",
                "publish_tf": "true",
                "initial_reset": "false",
            }.items(),
        ))

    if as_bool(LaunchConfiguration("imu").perform(context)):
        actions.append(Node(
            package="ds_nav_imu",
            executable="hipnuc_imu_node",
            name="hipnuc_imu_node",
            output="screen",
            parameters=[
                os.path.join(share, "config", "hipnuc", "hipnuc.yaml"),
                {"serial_port": LaunchConfiguration("serial_port").perform(context)},
            ],
        ))

    if as_bool(LaunchConfiguration("lidar").perform(context)):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "lidar_pointlio.launch.py")),
            launch_arguments={
                "driver": "dual_merged" if not use_sim_time else "none",
                "pointlio_config": "mid360_dual.yaml",
                "use_sim_time": str(use_sim_time).lower(),
            }.items(),
        ))

    if as_bool(LaunchConfiguration("openvins").perform(context)):
        config_path = LaunchConfiguration("openvins_config").perform(context) or os.path.join(
            share, "config", "openvins", "d435_hipnuc", "estimator_config.yaml")
        actions.append(GroupAction([
            SetParameter(name="use_sim_time", value=use_sim_time),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("ov_msckf"), "launch", "subscribe.launch.py")),
                launch_arguments={
                    "namespace": "ov_msckf",
                    "config_path": config_path,
                    "use_stereo": "true",
                    "max_cameras": "2",
                    "verbosity": LaunchConfiguration("verbosity").perform(context),
                    "rviz_enable": "false",
                }.items(),
            ),
        ]))
    if as_bool(LaunchConfiguration("rviz").perform(context)):
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_vio",
            arguments=["-d", os.path.join(share, "rviz", "vio.rviz")],
            parameters=[{"use_sim_time": use_sim_time}],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="true", description="启动 D435（回放时 false）"),
        DeclareLaunchArgument("imu", default_value="true", description="启动 HIPNUC（回放时 false）"),
        DeclareLaunchArgument("openvins", default_value="true"),
        DeclareLaunchArgument("lidar", default_value="false", description="同时启动双雷达 Point-LIO 作参考"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("infra_profile", default_value="848,480,30"),
        DeclareLaunchArgument("openvins_config", default_value="", description="estimator_config.yaml 绝对路径，空 = 默认"),
        DeclareLaunchArgument("verbosity", default_value="INFO"),
        DeclareLaunchArgument("rviz", default_value="false", description="打开 rviz/vio.rviz"),
        OpaqueFunction(function=launch_setup),
    ])
