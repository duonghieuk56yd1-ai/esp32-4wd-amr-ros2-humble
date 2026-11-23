"""Chuyển gói LiDAR đã giải mã sang LaserScan và chọn stamp (không cần socket/ROS graph)."""

import math

import pytest

from amr_lidar_bridge.bridge_node import NS_PER_S, scan_to_msg, select_stamp_ns
from amr_lidar_bridge.packet import decode, encode

NOW_NS = 1_750_000_000_000_000_000


def packet(**kwargs):
    defaults = dict(seq=1, stamp_ns=NOW_NS - 120_000_000, ros_time=True, scan_time=0.14,
                    range_min=0.1, range_max=8.0)
    defaults.update(kwargs)
    ranges = defaults.pop('ranges', [1000, 0, 2500, 65535])
    return decode(encode(ranges, **defaults))


# ---------------------------------------------------------------------- LaserScan
def test_scan_to_msg_copies_geometry_and_ranges():
    pkt = packet(intensities=[10, 0, 255, 3])
    msg = scan_to_msg(pkt, 'laser_frame', NOW_NS)
    assert msg.header.frame_id == 'laser_frame'
    assert msg.angle_min == 0.0
    assert msg.angle_increment == pytest.approx(math.pi / 2, rel=1e-6)
    assert msg.angle_max == pytest.approx(3 * math.pi / 2, rel=1e-6)
    assert msg.time_increment == 0.0
    assert msg.scan_time == pytest.approx(0.14, rel=1e-6)
    assert (msg.range_min, msg.range_max) == pytest.approx((0.1, 8.0))
    assert msg.ranges[0] == pytest.approx(1.0)
    assert math.isinf(msg.ranges[1])            # 0 mm -> +inf (không có phản hồi)
    assert msg.ranges[2] == pytest.approx(2.5)
    assert msg.ranges[3] == pytest.approx(65.535)
    assert list(msg.intensities) == [10.0, 0.0, 255.0, 3.0]


def test_scan_to_msg_without_intensity_and_range_override():
    msg = scan_to_msg(packet(), 'laser_frame', NOW_NS, range_min=0.15, range_max=6.0)
    assert len(msg.intensities) == 0
    assert (msg.range_min, msg.range_max) == pytest.approx((0.15, 6.0))


def test_scan_to_msg_splits_stamp():
    msg = scan_to_msg(packet(), 'laser_frame', 12 * NS_PER_S + 345)
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (12, 345)


# ---------------------------------------------------------------------- stamp
def test_uses_device_stamp_when_synced_and_close():
    pkt = packet()
    assert select_stamp_ns(NOW_NS, pkt, True, 1.0) == (pkt.stamp_ns, None)


def test_rejects_device_stamp_with_large_skew():
    pkt = packet(stamp_ns=NOW_NS - 3 * NS_PER_S)  # đồng hồ ESP32 chậm 3 s
    stamp, skew = select_stamp_ns(NOW_NS, pkt, True, 1.0)
    assert stamp == NOW_NS
    assert skew == pytest.approx(3.0)


def test_ignores_unsynced_or_disabled_device_stamp():
    assert select_stamp_ns(NOW_NS, packet(ros_time=False), True, 1.0) == (NOW_NS, None)
    assert select_stamp_ns(NOW_NS, packet(), False, 1.0) == (NOW_NS, None)


def test_time_offset_is_added():
    pkt = packet()
    stamp, _ = select_stamp_ns(NOW_NS, pkt, True, 1.0, time_offset=-0.05)
    assert stamp == pkt.stamp_ns - 50_000_000
    stamp, _ = select_stamp_ns(NOW_NS, pkt, False, 1.0, time_offset=0.02)
    assert stamp == NOW_NS + 20_000_000
