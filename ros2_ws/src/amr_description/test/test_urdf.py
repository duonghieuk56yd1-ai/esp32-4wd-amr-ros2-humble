"""Kiểm tra URDF sinh từ amr_3wd.urdf.xacro: cây TF, kích thước và quán tính."""

import importlib.util
import os
import xml.etree.ElementTree as ET

import pytest
import xacro

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XACRO_FILE = os.path.join(PKG_DIR, 'urdf', 'amr_3wd.urdf.xacro')
MESH_DIR = os.path.join(PKG_DIR, 'meshes')


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


# ------------------------------------------------------------- mesh từ FreeCAD
@pytest.fixture(scope='module')
def stl_tool():
    spec = importlib.util.spec_from_file_location(
        'stl_tool', os.path.join(PKG_DIR, 'scripts', 'stl_tool.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def mesh_visuals(robot):
    return {link.get('name'): link.find('visual/geometry/mesh')
            for link in robot.findall('link') if link.find('visual/geometry/mesh') is not None}


def test_default_model_uses_no_meshes():
    assert mesh_visuals(load()) == {}


def test_use_meshes_swaps_only_visuals():
    plain, meshed = load(), load(use_meshes='true')
    meshes = mesh_visuals(meshed)
    assert {k: m.get('filename') for k, m in meshes.items()} == {
        'base_link': 'package://amr_description/meshes/chassis.stl',
        'left_wheel_link': 'package://amr_description/meshes/wheel.stl',
        'right_wheel_link': 'package://amr_description/meshes/wheel.stl',
        'laser_frame': 'package://amr_description/meshes/lidar.stl',
    }
    for mesh in meshes.values():
        assert floats(mesh.get('scale')) == pytest.approx([0.001] * 3)
    # mỗi link vẫn chỉ 1 visual dạng mesh; khớp, collision và quán tính giữ nguyên
    for name in meshes:
        link = next(lk for lk in meshed.findall('link') if lk.get('name') == name)
        assert len(link.findall('visual')) == 1
    assert ET.tostring(plain.find('joint')) == ET.tostring(meshed.find('joint'))
    for tag in ('joint', 'link/collision', 'link/inertial'):
        assert [ET.tostring(e) for e in plain.findall(tag)] == \
            [ET.tostring(e) for e in meshed.findall(tag)], tag


def test_chassis_mesh_sits_on_ground_and_right_wheel_is_mirrored():
    robot = load(use_meshes='true', wheel_radius=0.04)
    links = {lk.get('name'): lk for lk in robot.findall('link')}
    chassis = links['base_link'].find('visual/origin')
    # chassis.stl có gốc trên mặt đất: lệch xuống đúng bán kính bánh so với base_link
    assert floats(chassis.get('xyz')) == pytest.approx([0.0, 0.0, -0.04])
    left = floats(links['left_wheel_link'].find('visual/origin').get('rpy'))
    right = floats(links['right_wheel_link'].find('visual/origin').get('rpy'))
    assert left == pytest.approx([0.0, 0.0, 0.0])
    assert right == pytest.approx([0.0, 0.0, 3.141592653589793])


def test_mesh_scale_override():
    for mesh in mesh_visuals(load(use_meshes='true', mesh_scale=1.0)).values():
        assert floats(mesh.get('scale')) == pytest.approx([1.0] * 3)


def test_shipped_meshes_follow_conventions(stl_tool):
    """meshes/*.stl (mẫu hoặc vẽ bằng FreeCAD) phải đúng đơn vị mm và gốc toạ độ."""
    robot = load()
    links = {lk.get('name'): lk for lk in robot.findall('link')}
    wheel_radius = float(links['left_wheel_link'].find('collision/geometry/cylinder')
                         .get('radius'))
    for name in ('chassis', 'wheel', 'lidar'):
        info, warnings = stl_tool.check_mesh(os.path.join(MESH_DIR, f'{name}.stl'))
        assert warnings == [], (name, warnings)
        if name == 'wheel':
            # bánh vẽ trong FreeCAD phải cùng bán kính với URDF / firmware (sai lệch < 1 mm)
            assert info['wheel_radius_m'] == pytest.approx(wheel_radius, abs=0.001)


def test_stl_tool_reads_ascii_and_flags_wrong_units(stl_tool, tmp_path):
    tris = stl_tool.cylinder(32.5, 26.0, axis='y')
    path = tmp_path / 'wheel.stl'
    with open(path, 'w', encoding='ascii') as f:
        f.write('solid w\n')
        for tri in tris:
            f.write(' facet normal 0 0 0\n  outer loop\n')
            for v in tri:
                f.write('   vertex %f %f %f\n' % v)
            f.write('  endloop\n endfacet\n')
        f.write('endsolid w\n')
    info, warnings = stl_tool.check_mesh(str(path))
    assert info['triangles'] == len(tris) and warnings == []
    assert info['wheel_radius_m'] == pytest.approx(0.0325, abs=1e-6)
    # cùng file nhưng khai báo STL theo mét -> 32 m: phải cảnh báo sai đơn vị
    _, warnings = stl_tool.check_mesh(str(path), scale=1.0)
    assert any('mesh_scale' in w for w in warnings)
    # bánh đặt lệch gốc 5 mm -> cảnh báo gốc toạ độ
    shifted = stl_tool.cylinder(32.5, 26.0, axis='y', center=(5.0, 0.0, 0.0))
    stl_tool.write_stl(str(tmp_path / 'wheel_off.stl'), shifted)
    _, warnings = stl_tool.check_mesh(str(tmp_path / 'wheel_off.stl'))
    assert any('tâm bánh' in w for w in warnings)
