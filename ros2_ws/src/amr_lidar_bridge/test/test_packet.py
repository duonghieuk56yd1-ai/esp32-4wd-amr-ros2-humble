"""
Bộ giải mã gói LiDAR UDP "AMRL" v1 (amr_lidar_bridge.packet).

Hai chiều tương thích với firmware:
  * fixtures/scan_ydlidar_x2_v1.bin do bộ đóng gói C++ của firmware tạo ra;
  * GOLDEN_PACKET do encode() Python tạo ra, firmware giải mã đúng gói này trong
    firmware_esp32/test/test_scan_packet (kPythonPacket).
"""

import math
import os
import random
import struct

import pytest

from amr_lidar_bridge.packet import (decode, encode, FLAG_INTENSITY, FLAG_ROS_TIME, HEADER,
                                     MAX_UDP_PAYLOAD, PacketError)

FIXTURE = os.path.join(os.path.dirname(__file__), 'fixtures', 'scan_ydlidar_x2_v1.bin')

GOLDEN_KWARGS = dict(seq=0xDEADBEEF, stamp_ns=1_700_000_000_123_456_789, ros_time=True, model=2,
                     scan_time=0.19, range_min=0.08, range_max=8.0,
                     intensities=[0, 10, 255, 1, 0, 128, 7, 0])
GOLDEN_RANGES = [0, 1000, 65535, 1, 0, 2500, 300, 0]
GOLDEN_PACKET = bytes.fromhex(
    '414d524c01020300efbeadde15cd853dfe9c97175c8f423e00000000db0f493f'
    '0ad7a33d00000041080005000000e803ffff01000000c4092c010000000aff01'
    '00800700')


@pytest.fixture
def firmware_packet():
    with open(FIXTURE, 'rb') as f:
        return f.read()


def with_header(packet, **fields):
    """Sửa vài trường header của một gói hợp lệ."""
    names = ('magic', 'version', 'model', 'flags', 'seq', 'stamp_ns', 'scan_time', 'angle_min',
             'angle_increment', 'range_min', 'range_max', 'num_ranges', 'num_valid')
    values = dict(zip(names, HEADER.unpack_from(packet)))
    values.update(fields)
    return HEADER.pack(*(values[n] for n in names)) + packet[HEADER.size:]


# ----------------------------------------------------------------------------- định dạng
def test_header_size_matches_firmware():
    assert HEADER.size == 44
    assert HEADER.format == '<4sBBHIQfffffHH'


def test_python_encoder_golden_bytes():
    """Gói này được nhúng nguyên văn trong unit test firmware: không được đổi."""
    assert encode(GOLDEN_RANGES, **GOLDEN_KWARGS) == GOLDEN_PACKET


def test_decode_golden_packet():
    pkt = decode(GOLDEN_PACKET)
    assert pkt.model == 2 and pkt.model_name == 'Camsense X1'
    assert pkt.flags == FLAG_INTENSITY | FLAG_ROS_TIME and pkt.ros_time
    assert pkt.seq == 0xDEADBEEF
    assert pkt.stamp_ns == 1_700_000_000_123_456_789
    assert pkt.scan_time == pytest.approx(0.19, rel=1e-6)
    assert pkt.angle_increment == pytest.approx(math.pi / 4, rel=1e-6)
    assert pkt.num_ranges == 8 and pkt.num_valid == 5
    assert list(pkt.ranges_mm) == GOLDEN_RANGES
    assert list(pkt.intensities) == GOLDEN_KWARGS['intensities']


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


def test_accepts_bytearray_and_memoryview(firmware_packet):
    for buf in (bytearray(firmware_packet), memoryview(firmware_packet)):
        pkt = decode(buf)
        assert pkt.num_ranges == 360 and pkt.seq == 42


# ------------------------------------------------------------------------ giá trị
def test_ranges_m_units_and_no_return():
    pkt = decode(encode([0, 1, 1000, 65535]))
    r = pkt.ranges_m()
    assert math.isinf(r[0]) and r[0] > 0      # 0 mm = không phản hồi -> +inf (REP-117)
    assert r[1] == pytest.approx(0.001)
    assert r[2] == pytest.approx(1.0)
    assert r[3] == pytest.approx(65.535)
    assert pkt.num_valid == 3


def test_all_rays_without_return():
    pkt = decode(encode([0] * 360))
    assert pkt.num_valid == 0
    assert all(math.isinf(r) for r in pkt.ranges_m())


def test_flags_and_model_names():
    assert not decode(encode([1])).ros_time
    assert decode(encode([1], ros_time=True)).ros_time
    assert decode(encode([1], intensities=[5])).flags == FLAG_INTENSITY
    assert decode(encode([1], model=1)).model_name == 'YDLIDAR X2'
    assert decode(encode([1], model=2)).model_name == 'Camsense X1'
    assert decode(encode([1], model=7)).model_name == 'unknown(7)'


def test_angle_max_covers_full_circle():
    for n in (1, 90, 360, 476):
        pkt = decode(encode([1000] * n))
        assert pkt.angle_increment == pytest.approx(2 * math.pi / n, rel=1e-6)
        assert pkt.angle_max == pytest.approx(2 * math.pi * (n - 1) / n, rel=1e-6)


def test_max_rays_fit_one_udp_datagram():
    """44 + 476 * 3 = 1472 byte: tối đa không bị phân mảnh IP (giống firmware)."""
    assert len(encode([1000] * 476, intensities=[1] * 476)) == MAX_UDP_PAYLOAD == 1472


def test_fuzz_round_trip():
    rng = random.Random(2024)
    for _ in range(300):
        n = rng.randint(1, 476)
        ranges = [rng.choice((0, rng.randint(1, 65535))) for _ in range(n)]
        intensities = [rng.randint(0, 255) for _ in range(n)] if rng.random() < 0.5 else None
        kwargs = dict(seq=rng.getrandbits(32), stamp_ns=rng.getrandbits(64),
                      ros_time=rng.random() < 0.5, model=rng.randint(0, 255),
                      scan_time=rng.uniform(0.0, 1.0), intensities=intensities)
        pkt = decode(encode(ranges, **kwargs))
        assert list(pkt.ranges_mm) == ranges
        assert pkt.num_valid == sum(1 for r in ranges if r)
        assert (pkt.seq, pkt.stamp_ns, pkt.ros_time, pkt.model) == \
            (kwargs['seq'], kwargs['stamp_ns'], kwargs['ros_time'], kwargs['model'])
        assert pkt.intensities == (bytes(intensities) if intensities is not None else None)


# -------------------------------------------------------------------- gói hỏng
@pytest.mark.parametrize('corrupt', [
    lambda b: b[:-1],                        # sai độ dài
    lambda b: b + b'\x00',                   # thừa byte
    lambda b: b'XMRL' + b[4:],               # sai magic
    lambda b: b[:4] + bytes([2]) + b[5:],    # sai version
    lambda b: b[:20],                        # ngắn hơn header
    lambda b: b'',
])
def test_rejects_corrupt_packets(firmware_packet, corrupt):
    with pytest.raises(PacketError):
        decode(corrupt(firmware_packet))


def test_rejects_every_truncation(firmware_packet):
    for n in range(len(firmware_packet)):
        with pytest.raises(PacketError):
            decode(firmware_packet[:n])


def test_rejects_intensity_flag_mismatch(firmware_packet):
    flags = HEADER.unpack_from(firmware_packet)[3]
    with pytest.raises(PacketError):
        decode(with_header(firmware_packet, flags=flags & ~FLAG_INTENSITY))


@pytest.mark.parametrize('fields', [
    {'angle_increment': 0.0},
    {'angle_increment': -0.01},
    {'angle_increment': float('nan')},
    {'angle_increment': float('inf')},
    {'angle_min': float('nan')},
])
def test_rejects_invalid_angles(firmware_packet, fields):
    with pytest.raises(PacketError):
        decode(with_header(firmware_packet, **fields))


def test_rejects_zero_rays():
    header = HEADER.pack(b'AMRL', 1, 1, 0, 0, 0, 0.1, 0.0, 0.1, 0.1, 8.0, 0, 0)
    with pytest.raises(PacketError):
        decode(header)


def test_encode_round_trip_without_intensity():
    ranges = [0, 1234, 65535] + [800] * 357
    pkt = decode(encode(ranges, seq=7, stamp_ns=5, scan_time=0.2))
    assert pkt.intensities is None
    assert not pkt.ros_time
    assert list(pkt.ranges_mm) == ranges
    assert pkt.num_valid == 359
    assert pkt.ranges_m()[1] == pytest.approx(1.234)


def test_encode_validates_input():
    with pytest.raises(ValueError):
        encode([1, 2, 3], intensities=[1, 2])
    with pytest.raises(struct.error):
        encode([70000])  # > uint16
