"""Run detection independently of CARLA input publishers."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('carla_camera_pipeline'),
                          'config', 'detection.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        Node(package='carla_camera_pipeline', executable='object_detection',
             name='object_detection', parameters=[LaunchConfiguration('params_file')],
             output='screen'),
    ])
