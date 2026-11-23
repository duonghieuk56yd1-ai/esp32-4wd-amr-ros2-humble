"""
Quét bản đồ 2D bằng slam_toolbox (online async) qua Wi-Fi + RViz2.

    ros2 launch amr_bringup slam.launch.py                 # gồm cả bringup (agent, LiDAR, URDF)
    ros2 launch amr_bringup slam.launch.py bringup:=false  # khi bringup.launch.py đang chạy riêng

Lưu bản đồ khi quét xong (đặt tên khớp mặc định của navigation.launch.py):

    mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/amr_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('amr_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('bringup', default_value='true',
                              description='Chạy kèm bringup.launch.py (agent, LiDAR, URDF)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Mở RViz2 (slam.rviz)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(bringup_share, 'config',
                                       'mapper_params_online_async.yaml'),
            description='File tham số slam_toolbox'),

        # GroupAction: rviz:=false chỉ áp dụng cho bringup, không ghi đè 'rviz' của file này
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, 'launch', 'bringup.launch.py')),
                launch_arguments={'rviz': 'false', 'use_sim_time': use_sim_time}.items()),
        ], condition=IfCondition(LaunchConfiguration('bringup'))),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                LaunchConfiguration('slam_params_file'),
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(bringup_share, 'rviz', 'slam.rviz')],
            parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
