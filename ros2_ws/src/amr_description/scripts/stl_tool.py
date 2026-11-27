#!/usr/bin/env python3
"""
Công cụ STL cho amr_description (chỉ dùng thư viện chuẩn Python, không cần ROS).

Kiểm tra STL xuất từ FreeCAD có đúng đơn vị / gốc toạ độ / kích thước không:
    python3 scripts/stl_tool.py check meshes/wheel.stl meshes/chassis.stl meshes/lidar.stl

Tạo lại bộ STL mẫu (hình đơn giản, khớp kích thước mặc định trong URDF):
    python3 scripts/stl_tool.py placeholders meshes/

Quy ước (xem ros2_ws/docs/FREECAD_URDF.md): STL theo mm, +X = phía trước xe, +Z = hướng lên.
  chassis.stl : gốc trên MẶT ĐẤT, ngay dưới tâm trục 2 bánh chủ động
  wheel.stl   : gốc = tâm bánh, trục bánh theo Y (vẽ bánh TRÁI)
  lidar.stl   : gốc = tâm mặt phẳng quét, +X = hướng tia 0°
"""

import math
import os
import struct
import sys

# Kích thước mặc định (mm), trùng amr_3wd.urdf.xacro / robot_config.h
WHEEL_RADIUS = 32.5
WHEEL_WIDTH = 26.0
CHASSIS_LENGTH, CHASSIS_WIDTH, CHASSIS_HEIGHT = 220.0, 180.0, 70.0
CHASSIS_X_OFFSET = 60.0       # tâm thân xe nằm trước trục bánh
CHASSIS_CLEARANCE = 15.0      # khoảng sáng gầm
LIDAR_RADIUS, LIDAR_HEIGHT = 30.0, 25.0


# ----------------------------------------------------------------- đọc / ghi STL
def read_stl(path):
    """Trả về danh sách tam giác [(v1, v2, v3), ...] từ STL nhị phân hoặc ASCII."""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) >= 84:
        count = struct.unpack_from('<I', data, 80)[0]
        if len(data) == 84 + 50 * count:  # STL nhị phân: kích thước khớp chính xác
            tris = []
            for i in range(count):
                vals = struct.unpack_from('<12f', data, 84 + 50 * i)
                tris.append((vals[3:6], vals[6:9], vals[9:12]))
            return tris
    verts = []
    for line in data.decode('utf-8', errors='replace').splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == 'vertex':
            verts.append(tuple(float(v) for v in parts[1:]))
    if not verts or len(verts) % 3:
        raise ValueError(f'{path}: không đọc được STL (rỗng hoặc hỏng)')
    return [tuple(verts[i:i + 3]) for i in range(0, len(verts), 3)]


def write_stl(path, tris, name='amr'):
    """Ghi STL nhị phân (pháp tuyến tính từ thứ tự đỉnh, ngược chiều kim đồng hồ = mặt ngoài)."""
    with open(path, 'wb') as f:
        f.write(name.encode('ascii')[:80].ljust(80, b'\0'))
        f.write(struct.pack('<I', len(tris)))
        for a, b, c in tris:
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
            length = math.sqrt(sum(x * x for x in n)) or 1.0
            f.write(struct.pack('<12fH', *(x / length for x in n), *a, *b, *c, 0))


def bounds(tris):
    pts = [p for tri in tris for p in tri]
    lo = [min(p[i] for p in pts) for i in range(3)]
    hi = [max(p[i] for p in pts) for i in range(3)]
    return lo, hi


# ----------------------------------------------------------------- hình cơ bản (mm)
def box(cx, cy, cz, sx, sy, sz):
    x0, x1 = cx - sx / 2, cx + sx / 2
    y0, y1 = cy - sy / 2, cy + sy / 2
    z0, z1 = cz - sz / 2, cz + sz / 2
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    tris = []
    for a, b, c, d in faces:
        tris += [(v[a], v[b], v[c]), (v[a], v[c], v[d])]
    return tris


def cylinder(radius, length, axis='z', center=(0.0, 0.0, 0.0), segments=48):
    """Trụ kín, trục theo 'y' hoặc 'z'."""
    def pt(angle, h):
        a, b = radius * math.cos(angle), radius * math.sin(angle)
        if axis == 'y':  # (x, y, z) = (a, h, b): vòng tròn trong mặt XZ
            p = (a, h, b)
        else:
            p = (a, b, h)
        return tuple(p[i] + center[i] for i in range(3))

    lo, hi = -length / 2, length / 2
    c_lo = tuple(center[i] + (lo if (axis == 'y' and i == 1) or (axis == 'z' and i == 2) else 0)
                 for i in range(3))
    c_hi = tuple(center[i] + (hi if (axis == 'y' and i == 1) or (axis == 'z' and i == 2) else 0)
                 for i in range(3))
    tris = []
    for k in range(segments):
        a0, a1 = 2 * math.pi * k / segments, 2 * math.pi * (k + 1) / segments
        p0, p1, q0, q1 = pt(a0, lo), pt(a1, lo), pt(a0, hi), pt(a1, hi)
        if axis == 'y':  # vòng quay (x,z) ngược chiều quanh -Y -> đảo thứ tự để pháp tuyến hướng ra
            tris += [(p0, q1, p1), (p0, q0, q1), (c_lo, p0, p1), (c_hi, q1, q0)]
        else:
            tris += [(p0, p1, q1), (p0, q1, q0), (c_lo, p1, p0), (c_hi, q0, q1)]
    return tris


def make_placeholders(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    chassis = box(CHASSIS_X_OFFSET, 0.0, CHASSIS_CLEARANCE + CHASSIS_HEIGHT / 2,
                  CHASSIS_LENGTH, CHASSIS_WIDTH, CHASSIS_HEIGHT)
    wheel = cylinder(WHEEL_RADIUS, WHEEL_WIDTH, axis='y')
    # mặt ngoài bánh trái là +Y: thêm moay-ơ để thấy chiều lắp
    wheel += cylinder(WHEEL_RADIUS * 0.3, 4.0, axis='y', center=(0.0, WHEEL_WIDTH / 2 + 2.0, 0.0))
    lidar = cylinder(LIDAR_RADIUS, LIDAR_HEIGHT, axis='z')
    lidar += box(LIDAR_RADIUS + 4.0, 0.0, 0.0, 8.0, 8.0, 8.0)  # mũi chỉ hướng tia 0° (+X)
    for name, tris in (('chassis', chassis), ('wheel', wheel), ('lidar', lidar)):
        path = os.path.join(out_dir, f'{name}.stl')
        write_stl(path, tris, name=f'amr placeholder {name}')
        print(f'Đã ghi {path} ({len(tris)} tam giác)')


# ----------------------------------------------------------------- kiểm tra
def check_mesh(path, scale=0.001):
    """Trả về (thông tin, danh sách cảnh báo) cho một file STL."""
    tris = read_stl(path)
    lo, hi = bounds(tris)
    size = [(hi[i] - lo[i]) * scale for i in range(3)]
    center = [(hi[i] + lo[i]) / 2 * scale for i in range(3)]
    info = {'triangles': len(tris), 'size_m': size, 'center_m': center,
            'min_m': [v * scale for v in lo], 'max_m': [v * scale for v in hi]}
    warnings = []
    biggest = max(size)
    if biggest > 2.0:
        warnings.append(f'lớn {biggest:.1f} m: STL đã theo mét? -> chạy xacro với mesh_scale:=1.0')
    elif biggest < 0.005:
        warnings.append(f'nhỏ {biggest * 1000:.2f} mm: sai đơn vị khi xuất STL?')
    if len(tris) > 200000:
        warnings.append(f'{len(tris)} tam giác: quá nặng cho RViz, giảm độ mịn khi xuất (< 50k)')

    name = os.path.basename(path).lower()
    if name.startswith('wheel'):
        if abs(center[0]) > 0.002 or abs(center[2]) > 0.002:
            warnings.append('tâm bánh lệch gốc toạ độ > 2 mm theo X/Z: đặt tâm bánh tại gốc')
        if abs(size[0] - size[2]) > 0.002:
            warnings.append('bánh không tròn trong mặt XZ: trục bánh phải là trục Y')
        info['wheel_radius_m'] = size[2] / 2
    elif name.startswith('chassis'):
        if info['min_m'][2] < -0.001:
            warnings.append('thân xe có phần nằm dưới mặt đất (Z < 0): gốc phải nằm trên mặt đất')
    elif name.startswith('lidar'):
        if abs(center[0]) > 0.01 or abs(center[1]) > 0.01:
            warnings.append('tâm LiDAR lệch gốc > 10 mm theo X/Y: gốc phải ở tâm trục quay')
    return info, warnings


def main(argv):
    if len(argv) >= 2 and argv[0] == 'placeholders':
        make_placeholders(argv[1])
        return 0
    if len(argv) >= 2 and argv[0] == 'check':
        scale = 0.001
        files = []
        for a in argv[1:]:
            if a.startswith('--scale='):
                scale = float(a.split('=', 1)[1])
            else:
                files.append(a)
        bad = 0
        for path in files:
            info, warnings = check_mesh(path, scale)
            sx, sy, sz = (v * 1000 for v in info['size_m'])
            cx, cy, cz = (v * 1000 for v in info['center_m'])
            print(f'{path}: {info["triangles"]} tam giác')
            print(f'  kích thước X×Y×Z = {sx:.1f} × {sy:.1f} × {sz:.1f} mm, '
                  f'tâm hộp bao = ({cx:.1f}, {cy:.1f}, {cz:.1f}) mm')
            if 'wheel_radius_m' in info:
                print(f'  bán kính bánh ≈ {info["wheel_radius_m"]:.5f} m '
                      f'-> wheel_radius trong URDF / WHEEL_RADIUS_M')
            for w in warnings:
                print(f'  CẢNH BÁO: {w}')
            bad += bool(warnings)
            if not warnings:
                print('  OK')
        return 1 if bad else 0
    print(__doc__)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
