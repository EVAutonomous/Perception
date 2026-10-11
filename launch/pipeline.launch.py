"""Launch the CARLA camera publisher plus optional saver and rosbag recorder.

  ros2 launch carla_camera_pipeline pipeline.launch.py
  ros2 launch carla_camera_pipeline pipeline.launch.py save_images:=false record_bag:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('carla_camera_pipeline'), 'config', 'pipeline.yaml')

    params = LaunchConfiguration('params_file')
    topics = ['/carla/camera/rgb', '/carla/camera/camera_info', '/carla/ego/state', '/clock']

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('save_images', default_value='true'),
        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument('bag_path', default_value='carla_bag'),

        Node(package='carla_camera_pipeline', executable='camera_publisher',
             name='carla_camera_publisher', parameters=[params], output='screen'),

        Node(package='carla_camera_pipeline', executable='image_saver',
             name='carla_image_saver', parameters=[params], output='screen',
             condition=IfCondition(LaunchConfiguration('save_images'))),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap',
                 '-o', LaunchConfiguration('bag_path'), *topics],
            output='screen',
            condition=IfCondition(LaunchConfiguration('record_bag'))),
    ])
