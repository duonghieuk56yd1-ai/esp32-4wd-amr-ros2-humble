"""Kiểm tra cấu hình amr_bringup và độ khớp với URDF + firmware_esp32."""

import importlib.util
import math
import os
import re
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import pytest
import xacro
import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_DIR = os.path.abspath(os.path.join(PKG_DIR, '..', '..', '..'))
FIRMWARE_INCLUDE = os.path.join(REPO_DIR, 'firmware_esp32', 'include')
LAUNCH_FILES = ['bringup.launch.py', 'slam.launch.py', 'navigation.launch.py']


def load_yaml(name):
    with open(os.path.join(PKG_DIR, 'config', name), encoding='utf-8') as f:
        return yaml.safe_load(f)


def load_launch(name):
    path = os.path.join(PKG_DIR, 'launch', name)
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def launch_defaults(name):
    return {e.name: ''.join(s.perform(None) for s in e.default_value)
            for e in load_launch(name).entities if isinstance(e, DeclareLaunchArgument)
            and e.default_value is not None}


def firmware_defines():
    defines = {}
    for header in ('robot_config.h', 'secrets.h'):
        path = os.path.join(FIRMWARE_INCLUDE, header)
        if not os.path.isfile(path):
            pytest.skip('không có firmware_esp32 cạnh ros2_ws')
        with open(path, encoding='utf-8') as f:
            for line in f:
                m = re.match(r'\s*#define\s+(\w+)\s+("[^"]*"|[^\s/]+)', line)
                if m and m.group(1) not in defines:
                    value = m.group(2).strip('"')
                    defines[m.group(1)] = re.sub(r'(?<=\d)f$', '', value)
    return defines


def urdf():
    path = os.path.join(get_package_share_directory('amr_description'), 'urdf',
                        'amr_3wd.urdf.xacro')
    return ET.fromstring(xacro.process_file(path).toxml())


def ros_params(cfg):
    """Duyệt mọi khối ros__parameters (kể cả lồng như local_costmap/local_costmap)."""
    for key, value in cfg.items():
        if isinstance(value, dict):
            if 'ros__parameters' in value:
                yield key, value['ros__parameters']
            else:
                yield from ros_params(value)


@pytest.mark.parametrize('name', LAUNCH_FILES)
def test_launch_file_loads(name):
    assert load_launch(name).entities


@pytest.mark.parametrize('name', LAUNCH_FILES)
def test_includes_are_scoped(name):
    """Include ở cấp ngoài cùng sẽ làm rò launch_arguments (vd rviz:=false) ra launch cha."""
    for entity in load_launch(name).entities:
        assert not isinstance(entity, IncludeLaunchDescription), name


def test_nav2_params_for_wifi_diff_drive():
    nav2 = load_yaml('nav2_params.yaml')
    for node, params in ros_params(nav2):
        assert params.get('use_sim_time') is False, node
    assert nav2['amcl']['ros__parameters']['base_frame_id'] == 'base_footprint'
    assert nav2['amcl']['ros__parameters']['robot_model_type'] == \
        'nav2_amcl::DifferentialMotionModel'
    local = nav2['local_costmap']['local_costmap']['ros__parameters']
    glob = nav2['global_costmap']['global_costmap']['ros__parameters']
    controller = nav2['controller_server']['ros__parameters']['FollowPath']
    tolerances = {
        'amcl': nav2['amcl']['ros__parameters']['transform_tolerance'],
        'bt_navigator': nav2['bt_navigator']['ros__parameters']['transform_tolerance'],
        'controller': controller['transform_tolerance'],
        'local_costmap': local['transform_tolerance'],
        'global_costmap': glob['transform_tolerance'],
        'behavior_server': nav2['behavior_server']['ros__parameters']['transform_tolerance'],
    }
    assert set(tolerances.values()) == {0.5}, tolerances
    for params in (nav2['bt_navigator']['ros__parameters'], local, glob,
                   nav2['behavior_server']['ros__parameters']):
        assert params['robot_base_frame'] == 'base_footprint'
    assert local['global_frame'] == 'odom' and glob['global_frame'] == 'map'
    # Hệ số inflation của RPP phải bằng của costmap
    assert controller['inflation_cost_scaling_factor'] == \
        local['inflation_layer']['cost_scaling_factor']


def test_footprint_covers_robot_body():
    """robot_radius phải phủ mọi góc thân xe quanh base_link (tâm quay)."""
    robot = urdf()
    base = next(link for link in robot.findall('link') if link.get('name') == 'base_link')
    box = base.find('collision')
    size = [float(v) for v in box.find('geometry/box').get('size').split()]
    cx, cy, _ = (float(v) for v in box.find('origin').get('xyz').split())
    corner = max(math.hypot(cx + sx * size[0] / 2, cy + sy * size[1] / 2)
                 for sx in (-1, 1) for sy in (-1, 1))
    nav2 = load_yaml('nav2_params.yaml')
    for costmap in ('local_costmap', 'global_costmap'):
        radius = nav2[costmap][costmap]['ros__parameters']['robot_radius']
        assert radius >= corner, (costmap, radius, corner)


def test_slam_params():
    slam = load_yaml('mapper_params_online_async.yaml')['slam_toolbox']['ros__parameters']
    assert (slam['odom_frame'], slam['map_frame'], slam['base_frame']) == \
        ('odom', 'map', 'base_footprint')
    assert slam['scan_topic'] == '/scan'
    assert slam['mode'] == 'mapping'
    assert slam['transform_timeout'] >= 0.5
    assert slam['max_laser_range'] <= 8.0


def test_matches_firmware():
    fw = firmware_defines()
    robot = urdf()
    joints = {j.get('name'): j for j in robot.findall('joint')}
    assert set(joints) >= {fw['LEFT_WHEEL_JOINT_NAME'], fw['RIGHT_WHEEL_JOINT_NAME']}
    left_y = float(joints[fw['LEFT_WHEEL_JOINT_NAME']].find('origin').get('xyz').split()[1])
    assert 2 * left_y == pytest.approx(float(fw['WHEEL_SEPARATION_M']))
    wheel = next(link for link in robot.findall('link') if link.get('name') == 'left_wheel_link')
    radius = float(wheel.find('collision/geometry/cylinder').get('radius'))
    assert radius == pytest.approx(float(fw['WHEEL_RADIUS_M']))

    assert fw['ODOM_FRAME_ID'] == 'odom' and fw['BASE_FRAME_ID'] == 'base_footprint'

    bridge_cfg = os.path.join(get_package_share_directory('amr_lidar_bridge'), 'config',
                              'lidar_bridge.yaml')
    with open(bridge_cfg, encoding='utf-8') as f:
        bridge = yaml.safe_load(f)['lidar_bridge']['ros__parameters']
    defaults = launch_defaults('bringup.launch.py')
    assert bridge['udp_port'] == int(fw['LIDAR_UDP_PORT']) == int(defaults['lidar_port'])
    assert int(defaults['agent_port']) == int(fw['AGENT_PORT'])

    nav2 = load_yaml('nav2_params.yaml')
    smoother = nav2['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][0] <= float(fw['MAX_LINEAR_VEL_MPS'])
    assert smoother['max_velocity'][2] <= float(fw['MAX_ANGULAR_VEL_RAD_S'])
    rpp = nav2['controller_server']['ros__parameters']['FollowPath']
    assert rpp['desired_linear_vel'] <= float(fw['MAX_LINEAR_VEL_MPS'])
