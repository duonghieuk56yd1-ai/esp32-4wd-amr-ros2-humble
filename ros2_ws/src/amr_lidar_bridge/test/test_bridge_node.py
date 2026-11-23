"""Kiểm tra node lidar_bridge: gửi gói UDP thật tới node và đọc /scan."""

import math
import os
import socket
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from amr_lidar_bridge.bridge_node import LidarUdpBridge
from amr_lidar_bridge.packet import encode

FIXTURE = os.path.join(os.path.dirname(__file__), 'fixtures', 'scan_ydlidar_x2_v1.bin')


def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


@pytest.fixture
def bridge():
    context = Context()
    rclpy.init(context=context, domain_id=91)
    port = free_udp_port()
    node = LidarUdpBridge(context=context, parameter_overrides=[
        Parameter('udp_port', value=port), Parameter('bind_address', value='127.0.0.1')])
    listener = rclpy.create_node('scan_listener', context=context)
    received = []
    # Subscriber best effort (giống slam_toolbox/Nav2) vẫn nhận được từ publisher reliable
    listener.create_subscription(LaserScan, 'scan', received.append, qos_profile_sensor_data)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(listener)
    yield port, executor, received
    executor.shutdown()
    node.destroy_node()
    listener.destroy_node()
    rclpy.shutdown(context=context)


def send_until_received(port, payload, executor, received, timeout=10.0):
    before = len(received)
    deadline = time.monotonic() + timeout
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
        while time.monotonic() < deadline and len(received) == before:
            tx.sendto(payload, ('127.0.0.1', port))
            executor.spin_once(timeout_sec=0.1)
    assert len(received) > before, 'không nhận được /scan'
    return received[-1]


def test_publishes_laser_scan(bridge):
    port, executor, received = bridge
    with open(FIXTURE, 'rb') as f:
        payload = f.read()
    scan = send_until_received(port, payload, executor, received)
    assert scan.header.frame_id == 'laser_frame'
    assert len(scan.ranges) == 360
    assert scan.angle_min == 0.0
    assert scan.angle_max == pytest.approx(2.0 * math.pi * 359.0 / 360.0, rel=1e-6)
    assert scan.ranges[0] == pytest.approx(1.0)
    assert scan.ranges[90] == pytest.approx(0.5)
    assert math.isinf(scan.ranges[180])
    assert len(scan.intensities) == 360 and scan.intensities[0] == pytest.approx(10.0)
    # Stamp trong fixture (năm 2023) lệch quá max_stamp_skew -> dùng giờ nhận gói
    assert abs(scan.header.stamp.sec - time.time()) < 5.0


def test_uses_device_stamp_when_synced(bridge):
    port, executor, received = bridge
    stamp_ns = time.time_ns() - 150_000_000  # vòng quét bắt đầu 150 ms trước
    payload = encode([1000] * 360, seq=1, stamp_ns=stamp_ns, ros_time=True, scan_time=0.14)
    scan = send_until_received(port, payload, executor, received)
    assert scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec == stamp_ns
    assert len(scan.intensities) == 0
