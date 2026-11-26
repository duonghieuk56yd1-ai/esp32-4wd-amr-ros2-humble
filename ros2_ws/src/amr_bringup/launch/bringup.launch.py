"""
Khởi chạy phần "robot" trên Laptop.

  * micro-ROS agent UDP 8888  <-> ESP32: /cmd_vel, /odom, /joint_states, /tf
  * amr_lidar_bridge UDP 8889  -> /scan (frame laser_frame)
  * robot_state_publisher (URDF amr_description) + RViz2 (robot_view.rviz)

    ros2 launch amr_bringup bringup.launch.py
    ros2 launch amr_bringup bringup.launch.py agent:=docker rviz:=false

agent:=auto (mặc định) dùng gói ROS micro_ros_agent nếu đã build trong workspace,
không có thì chạy Docker image microros/micro-ros-agent:humble; agent:=none nếu
bạn tự chạy agent ở terminal khác.
"""

import os
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

AGENT_DOCKER_IMAGE = 'microros/micro-ros-agent:humble'
AGENT_DOCKER_NAME = 'amr_micro_ros_agent'


def _package_exists(name):
    try:
        get_package_share_directory(name)
        return True
    except PackageNotFoundError:
        return False


def _docker_stop_agent(*_args, **_kwargs):
    """
    Dừng container agent (nếu đang chạy).

    Ctrl+C chỉ tới được tiến trình `docker run`, không tới agent bên trong container,
    nên khi launch tắt phải chủ động `docker stop` (SIGTERM, sau 2 s là SIGKILL).
    """
    try:
        subprocess.run(['docker', 'stop', '-t', '2', AGENT_DOCKER_NAME],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       timeout=20, check=False)
    except (OSError, subprocess.TimeoutExpired):
        pass
    return []


def _micro_ros_agent(context):
    mode = LaunchConfiguration('agent').perform(context).strip().lower()
    port = LaunchConfiguration('agent_port').perform(context)
    verbose = LaunchConfiguration('agent_verbose').perform(context)

    if mode == 'none':
        return [LogInfo(msg=f'agent:=none -> hãy tự chạy micro-ROS agent UDP cổng {port}')]

    has_ros_agent = _package_exists('micro_ros_agent')
    if mode == 'ros' or (mode == 'auto' and has_ros_agent):
        if not has_ros_agent:
            return [LogInfo(msg='[LỖI] Không tìm thấy gói micro_ros_agent: build nó trong '
                                'workspace (xem ros2_ws/README.md) hoặc dùng agent:=docker')]
        return [Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['udp4', '--port', port, f'-v{verbose}'])]

    if mode == 'docker' or (mode == 'auto' and shutil.which('docker')):
        _docker_stop_agent()  # dọn container còn sót từ lần chạy trước (nếu có)
        return [
            LogInfo(msg=f'Chạy micro-ROS agent bằng Docker ({AGENT_DOCKER_IMAGE}), UDP {port}'),
            # --net=host để ESP32 tới được UDP trên Laptop; --init dọn tiến trình con.
            ExecuteProcess(
                cmd=['docker', 'run', '--rm', '--init', '--net=host', '--name', AGENT_DOCKER_NAME,
                     AGENT_DOCKER_IMAGE, 'udp4', '--port', port, f'-v{verbose}'],
                name='micro_ros_agent_docker',
                output='screen'),
            RegisterEventHandler(OnShutdown(on_shutdown=[
                OpaqueFunction(function=_docker_stop_agent)])),
        ]

    return [LogInfo(msg='[LỖI] Không có micro_ros_agent (gói ROS) lẫn docker. Xem phần '
                        '"micro-ROS agent" trong ros2_ws/README.md, hoặc chạy agent:=none '
                        'nếu đã tự khởi động agent.')]


def generate_launch_description():
    description_share = get_package_share_directory('amr_description')
    bridge_share = get_package_share_directory('amr_lidar_bridge')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'agent', default_value='auto', choices=['auto', 'ros', 'docker', 'none'],
            description='Cách chạy micro-ROS agent'),
        DeclareLaunchArgument('agent_port', default_value='8888',
                              description='Cổng UDP micro-ROS agent (AGENT_PORT của ESP32)'),
        DeclareLaunchArgument('agent_verbose', default_value='4',
                              description='Mức log của agent (0-6)'),
        DeclareLaunchArgument('lidar_port', default_value='8889',
                              description='Cổng UDP nhận gói LiDAR (LIDAR_UDP_PORT của ESP32)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Mở RViz2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_meshes', default_value='false',
                              description='true: hình 3D từ amr_description/meshes (FreeCAD)'),

        OpaqueFunction(function=_micro_ros_agent),

        Node(
            package='amr_lidar_bridge',
            executable='lidar_bridge',
            name='lidar_bridge',
            output='screen',
            parameters=[
                os.path.join(bridge_share, 'config', 'lidar_bridge.yaml'),
                {
                    'udp_port': ParameterValue(LaunchConfiguration('lidar_port'), value_type=int),
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                },
            ]),

        # robot_state_publisher + RViz2 (robot_view.rviz). GroupAction giữ launch_arguments
        # trong phạm vi riêng (IncludeLaunchDescription của Humble không tự tách phạm vi).
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(description_share, 'launch', 'display.launch.py')),
                launch_arguments={
                    'rviz': LaunchConfiguration('rviz'),
                    'preview': 'false',
                    'use_sim_time': use_sim_time,
                    'use_meshes': LaunchConfiguration('use_meshes'),
                }.items()),
        ]),
    ])
