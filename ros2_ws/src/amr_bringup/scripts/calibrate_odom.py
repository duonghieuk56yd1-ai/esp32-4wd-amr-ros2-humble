#!/usr/bin/env python3
"""
Đo /odom để hiệu chỉnh WHEEL_RADIUS_M và WHEEL_SEPARATION_M (Bước 2 trong README).

    ros2 run amr_bringup calibrate_odom

Đi thẳng: đặt robot ở vạch xuất phát, chạy node, lái thẳng một đoạn (vd 1 m), dừng hẳn,
nhấn Ctrl+C rồi nhập quãng đường đo bằng thước -> WHEEL_RADIUS_M mới.

Quay tại chỗ: chạy lại node, quay tại chỗ đúng N vòng (robot nhìn lại đúng vạch mốc),
nhấn Ctrl+C rồi nhập N -> WHEEL_SEPARATION_M mới. Hiệu chỉnh bán kính bánh trước.

Có thể truyền sẵn giá trị thật để không phải nhập:
    ros2 run amr_bringup calibrate_odom --ros-args -p real_distance:=1.0
    ros2 run amr_bringup calibrate_odom --ros-args -p real_turns:=5.0
"""

import math
import os
import re

TWO_PI = 2.0 * math.pi
DEFAULT_WHEEL_RADIUS = 0.0325
DEFAULT_WHEEL_SEPARATION = 0.18


def wrap_angle(angle):
    """Đưa góc về (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(z, w):
    """Góc yaw của quaternion quay quanh trục z (robot chạy trên mặt phẳng)."""
    return 2.0 * math.atan2(z, w)


class OdomTracker:
    """Quãng đường thẳng và góc quay tích luỹ (không bị quấn về ±180°) kể từ mẫu đầu tiên."""

    def __init__(self):
        self.start = None
        self.x = 0.0
        self.y = 0.0
        self.last_yaw = 0.0
        self.total_yaw = 0.0
        self.samples = 0

    def add(self, x, y, yaw):
        """Thêm một mẫu odometry (m, m, rad)."""
        if self.start is None:
            self.start = (x, y)
        else:
            # /odom 30 Hz: giữa 2 mẫu robot quay << 180° nên wrap_angle cho đúng góc đã quay
            self.total_yaw += wrap_angle(yaw - self.last_yaw)
        self.x, self.y, self.last_yaw = x, y, yaw
        self.samples += 1

    @property
    def distance(self):
        """Khoảng cách thẳng từ điểm bắt đầu (m)."""
        if self.start is None:
            return 0.0
        return math.hypot(self.x - self.start[0], self.y - self.start[1])

    @property
    def turns(self):
        """Số vòng đã quay (dương = ngược chiều kim đồng hồ)."""
        return self.total_yaw / TWO_PI


def corrected_wheel_radius(wheel_radius, odom_distance, real_distance):
    """Bán kính mới: r_mới = r_cũ * quãng đường thật / quãng đường odom."""
    if odom_distance <= 0.0 or real_distance <= 0.0:
        raise ValueError('quãng đường phải > 0')
    return wheel_radius * real_distance / odom_distance


def corrected_wheel_separation(wheel_separation, odom_yaw, real_turns):
    """Khoảng cách bánh mới: L_mới = L_cũ * góc quay odom / góc quay thật."""
    real_yaw = TWO_PI * abs(real_turns)
    if real_yaw <= 0.0 or odom_yaw == 0.0:
        raise ValueError('góc quay phải khác 0')
    return wheel_separation * abs(odom_yaw) / real_yaw


def urdf_wheel_params(xacro_path):
    """Đọc wheel_radius / wheel_separation mặc định trong amr_3wd.urdf.xacro."""
    values = {'wheel_radius': DEFAULT_WHEEL_RADIUS,
              'wheel_separation': DEFAULT_WHEEL_SEPARATION}
    try:
        with open(xacro_path, encoding='utf-8') as f:
            text = f.read()
    except OSError:
        return values
    for name in values:
        m = re.search(r'<xacro:arg\s+name="%s"\s+default="([0-9.eE+-]+)"' % name, text)
        if m:
            values[name] = float(m.group(1))
    return values


def _ask_float(prompt):
    try:
        text = input(prompt).strip().replace(',', '.')
    except (EOFError, KeyboardInterrupt):
        print()
        return 0.0
    try:
        return float(text) if text else 0.0
    except ValueError:
        print(f'  "{text}" không phải số -> bỏ qua')
        return 0.0


def report(tracker, wheel_radius, wheel_separation, real_distance=0.0, real_turns=0.0,
           ask=True):
    """In kết quả và giá trị mới cho robot_config.h / URDF."""
    print('\n')
    print('================ KẾT QUẢ ĐO ODOMETRY ================')
    if tracker.samples < 2:
        print('Chưa nhận đủ /odom: robot đã kết nối micro-ROS agent chưa? '
              '(ros2 topic hz /odom)')
        return
    print(f'Quãng đường theo odom : {tracker.distance:.4f} m')
    print(f'Góc quay theo odom    : {math.degrees(tracker.total_yaw):+.1f}° '
          f'({tracker.turns:+.3f} vòng)')
    print(f'Thông số đang dùng    : WHEEL_RADIUS_M = {wheel_radius:.5f}, '
          f'WHEEL_SEPARATION_M = {wheel_separation:.4f}')
    print('-----------------------------------------------------')

    if real_distance <= 0.0 and ask and tracker.distance > 0.05:
        real_distance = _ask_float('Quãng đường THẬT đo bằng thước (m), Enter để bỏ qua: ')
    if real_distance > 0.0 and tracker.distance > 0.05:
        r_new = corrected_wheel_radius(wheel_radius, tracker.distance, real_distance)
        print(f'WHEEL_RADIUS_M mới = {wheel_radius:.5f} x {real_distance:.4f} / '
              f'{tracker.distance:.4f} = {r_new:.5f}')
        print(f'  robot_config.h  : #define WHEEL_RADIUS_M {r_new:.5f}f')
        print(f'  amr_3wd.urdf.xacro: <xacro:arg name="wheel_radius" default="{r_new:.5f}"/>')

    if real_turns <= 0.0 and ask and abs(tracker.turns) > 0.5:
        real_turns = _ask_float('Số vòng THẬT robot đã quay tại chỗ, Enter để bỏ qua: ')
    if real_turns > 0.0 and abs(tracker.turns) > 0.5:
        l_new = corrected_wheel_separation(wheel_separation, tracker.total_yaw, real_turns)
        print(f'WHEEL_SEPARATION_M mới = {wheel_separation:.4f} x '
              f'{abs(tracker.turns):.3f} / {abs(real_turns):.3f} = {l_new:.4f}')
        print(f'  robot_config.h  : #define WHEEL_SEPARATION_M {l_new:.4f}f')
        print(f'  amr_3wd.urdf.xacro: <xacro:arg name="wheel_separation" default="{l_new:.4f}"/>')
    print('Sửa cả 2 file (firmware_esp32/include/ và ros2_ws/src/amr_description/urdf/), nạp lại')
    print('firmware (pio run -t upload) rồi chạy lại bringup.')


def main(args=None):
    from ament_index_python.packages import get_package_share_directory
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.executors import ExternalShutdownException

    # Mặc định = thông số trong URDF (luôn giữ trùng robot_config.h của firmware)
    urdf = urdf_wheel_params(os.path.join(get_package_share_directory('amr_description'),
                                          'urdf', 'amr_3wd.urdf.xacro'))
    rclpy.init(args=args)
    node = rclpy.create_node('odom_calibration')
    wheel_radius = float(node.declare_parameter('wheel_radius', urdf['wheel_radius']).value)
    wheel_separation = float(
        node.declare_parameter('wheel_separation', urdf['wheel_separation']).value)
    real_distance = float(node.declare_parameter('real_distance', 0.0).value)
    real_turns = float(node.declare_parameter('real_turns', 0.0).value)
    tracker = OdomTracker()

    def on_odom(msg):
        p = msg.pose.pose
        tracker.add(p.position.x, p.position.y,
                    yaw_from_quaternion(p.orientation.z, p.orientation.w))

    def show():
        if tracker.samples:
            print(f'\rQuãng đường {tracker.distance:7.3f} m | góc quay '
                  f'{math.degrees(tracker.total_yaw):+8.1f}° ({tracker.turns:+6.2f} vòng)   ',
                  end='', flush=True)

    node.create_subscription(Odometry, 'odom', on_odom, 10)
    node.create_timer(0.2, show)
    print('Đang đo /odom. Lái robot (teleop / ros2 topic pub), dừng hẳn rồi nhấn Ctrl+C.')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    report(tracker, wheel_radius, wheel_separation, real_distance, real_turns)


if __name__ == '__main__':
    main()
