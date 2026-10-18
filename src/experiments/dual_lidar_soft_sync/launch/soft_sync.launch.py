from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'mock': ('true', bool), 'duration_s': ('8.0', float),
        'front_topic': ('input/front', str), 'back_topic': ('input/back', str),
        'max_delta_ms': ('5.0', float), 'queue_size': ('10', int),
        'mock_offset_ms': ('3.0', float), 'mock_jitter_ms': ('1.0', float),
        'mock_drop_every': ('0', int),
        'csv_path': ('/tmp/soft_sync_pairs.csv', str),
        'use_sim_time': ('false', bool),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v[0]) for k, v in defaults.items()],
        Node(package='dual_lidar_soft_sync', executable='soft_sync_demo',
             namespace='sync_compare/soft', output='screen',
             parameters=[{k: ParameterValue(LaunchConfiguration(k), value_type=v[1])
                          for k, v in defaults.items()}]),
    ])
