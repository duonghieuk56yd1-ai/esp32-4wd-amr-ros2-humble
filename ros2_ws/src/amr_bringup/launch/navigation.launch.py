"""
Nav2 cho robot vi sai 2 bánh chạy qua Wi-Fi + RViz2 sẵn sàng bấm điểm đích.

Bản đồ có sẵn (map_server + AMCL):
    ros2 launch amr_bringup navigation.launch.py map:=$HOME/maps/amr_map.yaml

Vừa quét bản đồ vừa dẫn đường (slam_toolbox thay cho map_server + AMCL):
    ros2 launch amr_bringup navigation.launch.py slam:=true

Trong RViz2: "2D Pose Estimate" đặt vị trí ban đầu (chế độ AMCL), rồi
"2D Goal Pose" / "Nav2 Goal" bấm điểm đích để xe tự chạy.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _localization(context, nav2_launch_dir, nav2_common_args):
    """Bản đồ có sẵn -> map_server + AMCL; mở rộng '~' vì bash không làm sau 'map:='."""
    if LaunchConfiguration('slam').perform(context).lower() in ('true', '1', 'yes'):
        return []
    map_yaml = os.path.abspath(os.path.expanduser(LaunchConfiguration('map').perform(context)))
    if not os.path.isfile(map_yaml):
        reason = (f'Không tìm thấy bản đồ "{map_yaml}". Quét bằng slam.launch.py rồi lưu: '
                  'ros2 run nav2_map_server map_saver_cli -f ~/maps/amr_map '
                  '(hoặc chạy navigation.launch.py slam:=true)')
        return [LogInfo(msg=f'[LỖI] {reason}'), Shutdown(reason=reason)]
    return [
        LogInfo(msg=f'Dùng bản đồ {map_yaml}'),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_launch_dir, 'localization_launch.py')),
                launch_arguments={**nav2_common_args, 'map': map_yaml}.items()),
        ]),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('amr_bringup')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    slam = LaunchConfiguration('slam')

    nav2_common_args = {
        'use_sim_time': use_sim_time,
        'params_file': params_file,
        'autostart': autostart,
        'use_composition': 'False',
        'use_respawn': 'False',
        'log_level': log_level,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.join(os.path.expanduser('~'), 'maps', 'amr_map.yaml'),
            description='File bản đồ .yaml (khi slam:=false)'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(bringup_share, 'config', 'nav2_params.yaml'),
            description='File tham số Nav2'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='true: dùng slam_toolbox thay cho map_server + AMCL'),
        DeclareLaunchArgument('bringup', default_value='true',
                              description='Chạy kèm bringup.launch.py (agent, LiDAR, URDF)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Mở RViz2 (navigation.rviz)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Tự kích hoạt (activate) các node lifecycle của Nav2'),
        DeclareLaunchArgument('log_level', default_value='info'),

        # Mỗi include nằm trong GroupAction để launch_arguments (rviz, bringup...) không
        # ghi đè tham số cùng tên của file này (Include của Humble không tách phạm vi).
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, 'launch', 'bringup.launch.py')),
                launch_arguments={'rviz': 'false', 'use_sim_time': use_sim_time}.items()),
        ], condition=IfCondition(LaunchConfiguration('bringup'))),

        # Định vị: slam_toolbox (slam:=true) hoặc map_server + AMCL
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, 'launch', 'slam.launch.py')),
                launch_arguments={'bringup': 'false', 'rviz': 'false',
                                  'use_sim_time': use_sim_time}.items()),
        ], condition=IfCondition(slam)),
        OpaqueFunction(function=_localization, args=[nav2_launch_dir, nav2_common_args]),

        # Planner, controller (Regulated Pure Pursuit), behaviors, BT navigator, velocity smoother
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_launch_dir, 'navigation_launch.py')),
                launch_arguments=nav2_common_args.items()),
        ]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(bringup_share, 'rviz', 'navigation.rviz')],
            parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
