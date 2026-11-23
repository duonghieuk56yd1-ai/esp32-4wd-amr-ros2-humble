"""
Tên topic và frame phải thống nhất trên toàn repo.

Topic : /cmd_vel /odom /joint_states /tf /scan
Frame : map -> odom -> base_footprint -> base_link -> {bánh xe, caster, laser_frame}

Nguồn được đối chiếu: firmware_esp32/include/robot_config.h, URDF amr_description,
cấu hình amr_lidar_bridge, slam_toolbox, Nav2, các launch file và cấu hình RViz2.
"""

import os
import re
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from amr_lidar_bridge import bridge_node
import pytest
import xacro
import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.dirname(PKG_DIR)
REPO_DIR = os.path.abspath(os.path.join(SRC_DIR, '..', '..'))
ROBOT_CONFIG = os.path.join(REPO_DIR, 'firmware_esp32', 'include', 'robot_config.h')

RVIZ_FILES = {
    'robot_view.rviz': (os.path.join(SRC_DIR, 'amr_description', 'rviz', 'robot_view.rviz'),
                        'odom'),
    'slam.rviz': (os.path.join(PKG_DIR, 'rviz', 'slam.rviz'), 'map'),
    'navigation.rviz': (os.path.join(PKG_DIR, 'rviz', 'navigation.rviz'), 'map'),
}


def load_yaml(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def nav2_params():
    return load_yaml(os.path.join(PKG_DIR, 'config', 'nav2_params.yaml'))


def robot_config():
    if not os.path.isfile(ROBOT_CONFIG):
        pytest.skip('không có firmware_esp32 cạnh ros2_ws')
    defines = {}
    with open(ROBOT_CONFIG, encoding='utf-8') as f:
        for line in f:
            m = re.match(r'\s*#define\s+(\w+)\s+"([^"]*)"', line)
            if m:
                defines[m.group(1)] = m.group(2)
    return defines


def urdf_tree():
    path = os.path.join(get_package_share_directory('amr_description'), 'urdf',
                        'amr_3wd.urdf.xacro')
    robot = ET.fromstring(xacro.process_file(path).toxml())
    links = {link.get('name') for link in robot.findall('link')}
    joints = {j.get('name'): j for j in robot.findall('joint')}
    parent = {j.find('child').get('link'): j.find('parent').get('link') for j in joints.values()}
    return links, joints, parent


def ancestors(link, parent):
    chain = []
    while link in parent:
        link = parent[link]
        chain.append(link)
    return chain


def rviz_displays(path):
    """Mọi display (kể cả trong Group) của một file .rviz."""
    def walk(displays):
        for d in displays or []:
            yield d
            yield from walk(d.get('Displays'))
    cfg = load_yaml(path)['Visualization Manager']
    return cfg['Global Options']['Fixed Frame'], list(walk(cfg['Displays']))


# ------------------------------------------------------------------------- firmware
def test_firmware_topics_and_frames():
    fw = robot_config()
    assert fw['ROS_NAMESPACE'] == ''  # topic tương đối -> /cmd_vel, /odom, ...
    assert fw['TOPIC_CMD_VEL'] == 'cmd_vel'
    assert fw['TOPIC_ODOM'] == 'odom'
    assert fw['TOPIC_JOINT_STATES'] == 'joint_states'
    assert fw['TOPIC_TF'] == 'tf'
    assert fw['ODOM_FRAME_ID'] == 'odom'
    assert fw['BASE_FRAME_ID'] == 'base_footprint'


# ----------------------------------------------------------------------------- URDF
def test_urdf_tf_tree():
    links, joints, parent = urdf_tree()
    roots = links - set(parent)
    assert roots == {'base_footprint'}, 'gốc cây TF của robot phải là base_footprint'
    assert parent['base_link'] == 'base_footprint'
    for link in ('left_wheel_link', 'right_wheel_link', 'caster_wheel_link', 'laser_frame'):
        assert parent[link] == 'base_link', link
    assert joints['base_joint'].get('type') == 'fixed'
    assert joints['laser_joint'].get('type') == 'fixed'


def test_urdf_wheel_joints_match_firmware():
    fw = robot_config()
    _, joints, parent = urdf_tree()
    for name, link in ((fw['LEFT_WHEEL_JOINT_NAME'], 'left_wheel_link'),
                       (fw['RIGHT_WHEEL_JOINT_NAME'], 'right_wheel_link')):
        joint = joints[name]
        assert joint.get('type') == 'continuous'
        assert joint.find('child').get('link') == link
        # Trục quay +y: bánh lăn tới trước khi góc tăng (khớp quy ước firmware)
        assert [float(v) for v in joint.find('axis').get('xyz').split()] == [0.0, 1.0, 0.0]
    assert 'base_footprint' in ancestors('laser_frame', parent)


# -------------------------------------------------------------------- cầu nối LiDAR
def test_lidar_bridge_frame_and_topic():
    share = get_package_share_directory('amr_lidar_bridge')
    params = load_yaml(os.path.join(share, 'config', 'lidar_bridge.yaml'))
    assert set(params) == {'lidar_bridge'}  # trùng tên node trong bringup.launch.py
    assert params['lidar_bridge']['ros__parameters']['frame_id'] == 'laser_frame'
    _, _, parent = urdf_tree()
    assert 'laser_frame' in parent
    with open(bridge_node.__file__, encoding='utf-8') as f:
        source = f.read()
    assert "create_publisher(LaserScan, 'scan'" in source
    with open(os.path.join(PKG_DIR, 'launch', 'bringup.launch.py'), encoding='utf-8') as f:
        assert "name='lidar_bridge'" in f.read()


# ------------------------------------------------------------------------- SLAM
def test_slam_toolbox_names():
    slam = load_yaml(os.path.join(PKG_DIR, 'config', 'mapper_params_online_async.yaml'))
    assert set(slam) == {'slam_toolbox'}
    params = slam['slam_toolbox']['ros__parameters']
    assert params['scan_topic'] == '/scan'
    assert (params['map_frame'], params['odom_frame'], params['base_frame']) == \
        ('map', 'odom', 'base_footprint')
    with open(os.path.join(PKG_DIR, 'launch', 'slam.launch.py'), encoding='utf-8') as f:
        assert "name='slam_toolbox'" in f.read()


# ------------------------------------------------------------------------- Nav2
def test_nav2_frames():
    nav2 = nav2_params()
    amcl = nav2['amcl']['ros__parameters']
    assert (amcl['global_frame_id'], amcl['odom_frame_id'], amcl['base_frame_id']) == \
        ('map', 'odom', 'base_footprint')
    bt = nav2['bt_navigator']['ros__parameters']
    assert (bt['global_frame'], bt['robot_base_frame']) == ('map', 'base_footprint')
    local = nav2['local_costmap']['local_costmap']['ros__parameters']
    glob = nav2['global_costmap']['global_costmap']['ros__parameters']
    assert (local['global_frame'], local['robot_base_frame']) == ('odom', 'base_footprint')
    assert (glob['global_frame'], glob['robot_base_frame']) == ('map', 'base_footprint')
    behavior = nav2['behavior_server']['ros__parameters']
    assert (behavior['global_frame'], behavior['robot_base_frame']) == \
        ('odom', 'base_footprint')
    assert nav2['map_server']['ros__parameters'].get('frame_id', 'map') == 'map'


def test_nav2_topics():
    nav2 = nav2_params()
    assert nav2['amcl']['ros__parameters']['scan_topic'].lstrip('/') == 'scan'
    assert nav2['bt_navigator']['ros__parameters']['odom_topic'].lstrip('/') == 'odom'
    assert nav2['velocity_smoother']['ros__parameters']['odom_topic'].lstrip('/') == 'odom'
    for costmap in ('local_costmap', 'global_costmap'):
        params = nav2[costmap][costmap]['ros__parameters']
        for layer in params['plugins']:
            sources = params[layer].get('observation_sources', '').split()
            for source in sources:
                obs = params[layer][source]
                assert obs['topic'] == '/scan', (costmap, layer, source)
                assert obs['data_type'] == 'LaserScan'
    # Không remap /cmd_vel: velocity_smoother của Nav2 xuất bản đúng /cmd_vel cho ESP32
    with open(os.path.join(PKG_DIR, 'launch', 'navigation.launch.py'), encoding='utf-8') as f:
        assert 'cmd_vel' not in f.read()


# ------------------------------------------------------------------------- RViz2
@pytest.mark.parametrize('name', sorted(RVIZ_FILES))
def test_rviz_topics_and_fixed_frame(name):
    path, expected_fixed = RVIZ_FILES[name]
    fixed, displays = rviz_displays(path)
    assert fixed == expected_fixed
    by_class = {}
    for d in displays:
        by_class.setdefault(d['Class'], []).append(d)
    for scan in by_class.get('rviz_default_plugins/LaserScan', []):
        assert scan['Topic']['Value'] == '/scan'
    for odom in by_class.get('rviz_default_plugins/Odometry', []):
        assert odom['Topic']['Value'] == '/odom'
    for model in by_class.get('rviz_default_plugins/RobotModel', []):
        assert model['Description Topic']['Value'] == '/robot_description'
    assert by_class.get('rviz_default_plugins/LaserScan'), 'thiếu display LaserScan'
    assert by_class.get('rviz_default_plugins/RobotModel'), 'thiếu display RobotModel'
