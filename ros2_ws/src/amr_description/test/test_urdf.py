"""Kiểm tra URDF sinh từ amr_3wd.urdf.xacro: cây TF, kích thước và quán tính."""

import os
import xml.etree.ElementTree as ET

import pytest
import xacro

XACRO_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'urdf', 'amr_3wd.urdf.xacro')


def load(**mappings):
    doc = xacro.process_file(XACRO_FILE, mappings={k: str(v) for k, v in mappings.items()})
    return ET.fromstring(doc.toxml())


def floats(text):
    return [float(v) for v in text.split()]


def joints(robot):
    return {j.get('name'): j for j in robot.findall('joint')}


def joint_z_from_ground(robot, child):
    """Độ cao gốc frame `child` so với base_footprint (chỉ cộng tịnh tiến z)."""
    parents = {j.find('child').get('link'): j for j in robot.findall('joint')}
    z = 0.0
    while child in parents:
        joint = parents[child]
        z += floats(joint.find('origin').get('xyz'))[2]
        child = joint.find('parent').get('link')
    return z


def test_tf_tree():
    robot = load()
    tree = {j.find('child').get('link'): j.find('parent').get('link')
            for j in robot.findall('joint')}
    assert tree == {
        'base_link': 'base_footprint',
        'left_wheel_link': 'base_link',
        'right_wheel_link': 'base_link',
        'caster_wheel_link': 'base_link',
        'laser_frame': 'base_link',
        # link con chỉ để tô màu (mỗi link một material)
        'left_wheel_marker_link': 'left_wheel_link',
        'right_wheel_marker_link': 'right_wheel_link',
        'laser_mount_link': 'laser_frame',
    }


def test_wheel_joints_match_firmware_convention():
    robot = load()
    js = joints(robot)
    for side, sign in (('left', 1.0), ('right', -1.0)):
        joint = js[f'{side}_wheel_joint']
        assert joint.get('type') == 'continuous'
        # cả 2 bánh trục +Y: vận tốc dương = lăn về phía trước (như firmware)
        assert floats(joint.find('axis').get('xyz')) == [0.0, 1.0, 0.0]
        assert floats(joint.find('origin').get('xyz'))[1] == pytest.approx(sign * 0.09)
    for name in ('base_joint', 'caster_wheel_joint', 'laser_joint', 'laser_mount_joint',
                 'left_wheel_marker_joint', 'right_wheel_marker_joint'):
        assert js[name].get('type') == 'fixed'


def test_dimensions():
    robot = load()
    links = {link.get('name'): link for link in robot.findall('link')}
    wheel = links['left_wheel_link'].find('collision/geometry/cylinder')
    assert float(wheel.get('radius')) == pytest.approx(0.0325)
    body = links['base_link'].find('collision/geometry/box')
    assert floats(body.get('size')) == pytest.approx([0.22, 0.18, 0.07])
    caster = links['caster_wheel_link'].find('collision/geometry/sphere')
    # quả cầu caster chạm đất
    assert joint_z_from_ground(robot, 'caster_wheel_link') == pytest.approx(
        float(caster.get('radius')))
    assert joint_z_from_ground(robot, 'laser_frame') == pytest.approx(0.14)


def test_every_physical_link_has_visual_collision_inertial():
    robot = load()
    for link in robot.findall('link'):
        if link.get('name') == 'base_footprint':
            assert link.find('inertial') is None  # KDL không hỗ trợ quán tính ở root
            continue
        assert link.find('visual') is not None, link.get('name')
        assert link.find('collision') is not None, link.get('name')
        inertial = link.find('inertial')
        assert inertial is not None, link.get('name')
        assert float(inertial.find('mass').get('value')) > 0.0
        i = inertial.find('inertia')
        ixx, iyy, izz = (float(i.get(k)) for k in ('ixx', 'iyy', 'izz'))
        assert min(ixx, iyy, izz) > 0.0
        # bất đẳng thức tam giác của mômen quán tính chính
        assert ixx + iyy >= izz and iyy + izz >= ixx and ixx + izz >= iyy


def test_parameter_override():
    robot = load(lidar_height=0.15, wheel_separation=0.2)
    assert joint_z_from_ground(robot, 'laser_frame') == pytest.approx(0.15)
    left = joints(robot)['left_wheel_joint']
    assert floats(left.find('origin').get('xyz'))[1] == pytest.approx(0.1)


def test_lidar_below_roof_is_rejected():
    with pytest.raises(Exception):
        load(lidar_height=0.08)


def test_one_material_per_link():
    """RViz2 Humble tô mọi visual của một link bằng material của visual đầu tiên."""
    robot = load()
    for link in robot.findall('link'):
        materials = {v.find('material').get('name') for v in link.findall('visual')}
        assert len(materials) <= 1, (link.get('name'), materials)
