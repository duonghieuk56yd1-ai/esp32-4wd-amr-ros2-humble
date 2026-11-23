"""
Hiển thị robot amr_3wd trên RViz2.

Robot thật (mặc định): robot_state_publisher dựng TF các link từ URDF và từ
/joint_states do ESP32 gửi lên, nên bánh xe 3D quay đồng bộ với bánh thật;
TF odom -> base_footprint và /scan đến từ ESP32 / amr_lidar_bridge.

    ros2 launch amr_description display.launch.py

Xem mô hình khi chưa có robot (thêm thanh trượt quay bánh và TF odom tĩnh):

    ros2 launch amr_description display.launch.py preview:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_description')

    model = LaunchConfiguration('model')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    preview = LaunchConfiguration('preview')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', model]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=os.path.join(pkg_share, 'urdf', 'amr_3wd.urdf.xacro'),
            description='Đường dẫn file URDF/Xacro'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Mở RViz2'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share, 'rviz', 'robot_view.rviz'),
            description='File cấu hình RViz2'),
        DeclareLaunchArgument(
            'preview', default_value='false',
            description='true: không có robot -> joint_state_publisher_gui + TF odom tĩnh'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Dùng đồng hồ mô phỏng'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                # ESP32 gửi /joint_states 30 Hz -> cập nhật TF bánh xe cùng nhịp
                'publish_frequency': 30.0,
                'use_sim_time': use_sim_time,
            }]),

        # ---- Chế độ xem trước (không có ESP32) ----
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(preview)),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='preview_odom_tf',
            arguments=['--frame-id', 'odom', '--child-frame-id', 'base_footprint'],
            condition=IfCondition(preview)),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz)),
    ])
