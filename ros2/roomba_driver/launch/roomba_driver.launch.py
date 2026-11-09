"""Roomba base driver.

  ros2 launch roomba_driver roomba_driver.launch.py
  ros2 launch roomba_driver roomba_driver.launch.py params_file:=/path/to/my.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [FindPackageShare('roomba_driver'), 'config', 'roomba_driver.yaml'])),
        Node(package='roomba_driver', executable='roomba_driver', name='roomba_driver', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
    ])
