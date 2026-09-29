"""核心启动：livox_ros_driver2 + Point-LIO 放在同一个 component_container_mt（进程内零拷贝）。

各阶段 launch 都 include 本文件。常用参数：
  driver          single_mid360 | dual_merged | dual_split | none（none = 回放 bag，不启动驱动）
  pointlio_config config/point_lio/ 下的文件名（mid360_single.yaml / mid360_dual.yaml / mid360_hipnuc.yaml）
  lid_topic       覆盖 common.lid_topic（空 = 用配置文件）
  imu_topic       覆盖 common.imu_topic（空 = 用配置文件）
  blind_center    覆盖 preprocess.blind_center，格式 "x,y,z"（空 = 用配置文件）
  save_pcd        true 时退出（Ctrl-C）写 src/lio/point_lio/PCD/scans.pcd
  use_sim_time    回放 bag 时设 true，并用 ros2 bag play --clock
  rviz / rviz_config  rviz:=true 打开 RViz；rviz_config 为 rviz/ 下文件名或绝对路径（默认 handheld.rviz）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile

DRIVERS = ("single_mid360", "dual_merged", "dual_split", "none")


def as_bool(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes", "on")


def launch_setup(context):
    share = get_package_share_directory("handheld_bringup")
    driver = LaunchConfiguration("driver").perform(context)
    pointlio_config = LaunchConfiguration("pointlio_config").perform(context)
    lid_topic = LaunchConfiguration("lid_topic").perform(context)
    imu_topic = LaunchConfiguration("imu_topic").perform(context)
    blind_center = LaunchConfiguration("blind_center").perform(context)
    save_pcd = as_bool(LaunchConfiguration("save_pcd").perform(context))
    use_sim_time = as_bool(LaunchConfiguration("use_sim_time").perform(context))
    container_name = LaunchConfiguration("container_name").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    if not os.path.isabs(rviz_config):
        rviz_config = os.path.join(share, "rviz", rviz_config)

    if driver not in DRIVERS:
        raise RuntimeError(f"driver 必须是 {DRIVERS} 之一，收到 '{driver}'")
    if not os.path.isabs(pointlio_config):
        pointlio_config = os.path.join(share, "config", "point_lio", pointlio_config)
    if not os.path.isfile(pointlio_config):
        raise RuntimeError(f"Point-LIO 配置不存在: {pointlio_config}")

    intra = [{"use_intra_process_comms": True}]
    nodes = []
    if driver != "none":
        nodes.append(
            ComposableNode(
                package="livox_ros_driver2",
                plugin="livox_ros::DriverNode",
                name="livox_driver_node",
                parameters=[
                    ParameterFile(os.path.join(share, "config", "driver", f"{driver}.yaml"), allow_substs=True),
                    {"use_sim_time": use_sim_time},
                ],
                extra_arguments=intra,
            )
        )

    overrides = {"use_sim_time": use_sim_time}
    if lid_topic:
        overrides["common.lid_topic"] = lid_topic
    if imu_topic:
        overrides["common.imu_topic"] = imu_topic
    if blind_center:
        overrides["preprocess.blind_center"] = [float(v) for v in blind_center.split(",")]
    if save_pcd:
        # interval=-1：整段只在退出时写一个 scans.pcd，便于阶段 6 当先验地图
        overrides["pcd_save.pcd_save_en"] = True
        overrides["pcd_save.interval"] = -1
    nodes.append(
        ComposableNode(
            package="point_lio",
            plugin="point_lio::LaserMappingNode",
            name="laserMapping",
            parameters=[pointlio_config, overrides],
            extra_arguments=intra,
        )
    )

    container = ComposableNodeContainer(
        name=container_name,
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        composable_node_descriptions=nodes,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return [container, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("driver", default_value="dual_merged", description="|".join(DRIVERS)),
        DeclareLaunchArgument("pointlio_config", default_value="mid360_dual.yaml"),
        DeclareLaunchArgument("lid_topic", default_value=""),
        DeclareLaunchArgument("imu_topic", default_value=""),
        DeclareLaunchArgument("blind_center", default_value=""),
        DeclareLaunchArgument("save_pcd", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("container_name", default_value="handheld_lio_container"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value="handheld.rviz"),
        OpaqueFunction(function=launch_setup),
    ])
