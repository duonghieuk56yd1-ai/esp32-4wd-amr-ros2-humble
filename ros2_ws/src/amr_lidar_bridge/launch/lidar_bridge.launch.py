"""Chạy riêng cầu nối LiDAR UDP -> /scan."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory('amr_lidar_bridge'), 'config',
                          'lidar_bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('udp_port', default_value='8889',
                              description='Cổng UDP nhận gói LiDAR từ ESP32'),
        DeclareLaunchArgument('frame_id', default_value='laser_frame',
                              description='frame_id của sensor_msgs/LaserScan'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='amr_lidar_bridge',
            executable='lidar_bridge',
            name='lidar_bridge',
            output='screen',
            parameters=[params, {
                'udp_port': ParameterValue(LaunchConfiguration('udp_port'), value_type=int),
                'frame_id': LaunchConfiguration('frame_id'),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                               value_type=bool),
            }]),
    ])
