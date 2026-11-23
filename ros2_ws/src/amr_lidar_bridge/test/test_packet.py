"""Giải mã gói AMRL v1, dùng fixture do chính bộ đóng gói C++ của firmware tạo ra."""

import math
import os

import pytest

from amr_lidar_bridge.packet import decode, encode, HEADER, PacketError

FIXTURE = os.path.join(os.path.dirname(__file__), 'fixtures', 'scan_ydlidar_x2_v1.bin')


@pytest.fixture
def firmware_packet():
    with open(FIXTURE, 'rb') as f:
        return f.read()


def test_header_size_matches_firmware():
    assert HEADER.size == 44


def test_decode_firmware_fixture(firmware_packet):
    pkt = decode(firmware_packet)
    assert len(firmware_packet) == 1124
    assert pkt.model_name == 'YDLIDAR X2'
    assert pkt.seq == 42
    assert pkt.ros_time
    assert pkt.stamp_ns == 1700000000123456789
    assert pkt.scan_time == pytest.approx(1.0 / 7.0, rel=1e-6)
    assert pkt.num_ranges == 360
    assert pkt.num_valid == 321
    assert pkt.angle_min == 0.0
    assert pkt.angle_increment == pytest.approx(2.0 * math.pi / 360.0, rel=1e-6)
    assert pkt.angle_max == pytest.approx(2.0 * math.pi * 359.0 / 360.0, rel=1e-6)
    assert (pkt.range_min, pkt.range_max) == pytest.approx((0.1, 8.0))
    ranges = pkt.ranges_m()
    assert ranges[0] == pytest.approx(1.0)      # tường phía trước
    assert ranges[90] == pytest.approx(0.5)     # vật bên trái (+90 độ, quy ước ROS)
    assert ranges[270] == pytest.approx(3.0)    # bên phải
    assert math.isinf(ranges[180])              # phía sau không có phản hồi
    assert sum(math.isfinite(r) for r in ranges) == pkt.num_valid
    assert pkt.intensities is not None and len(pkt.intensities) == 360


@pytest.mark.parametrize('corrupt', [
    lambda b: b[:-1],                        # sai độ dài
    lambda b: b'XMRL' + b[4:],               # sai magic
    lambda b: b[:4] + bytes([2]) + b[5:],    # sai version
    lambda b: b[:20],                        # ngắn hơn header
])
def test_rejects_corrupt_packets(firmware_packet, corrupt):
    with pytest.raises(PacketError):
        decode(corrupt(firmware_packet))


def test_encode_round_trip_without_intensity():
    ranges = [0, 1234, 65535] + [800] * 357
    pkt = decode(encode(ranges, seq=7, stamp_ns=5, scan_time=0.2))
    assert pkt.intensities is None
    assert not pkt.ros_time
    assert list(pkt.ranges_mm) == ranges
    assert pkt.num_valid == 359
    assert pkt.ranges_m()[1] == pytest.approx(1.234)
