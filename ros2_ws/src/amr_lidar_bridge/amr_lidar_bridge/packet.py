"""
Giải mã / đóng gói gói LiDAR UDP do firmware_esp32 gửi (định dạng "AMRL" v1).

Module không phụ thuộc ROS để dễ kiểm thử. Định nghĩa gốc nằm ở
firmware_esp32/lib/amr_core/src/amr_scan_packet.h:

    header '<4sBBHIQfffffHH' (44 byte, little-endian)
    + uint16 ranges_mm[N]      (0 = không có phản hồi)
    + uint8 intensities[N]     (chỉ khi flags bit0 = 1)
"""

from __future__ import annotations

from array import array
from dataclasses import dataclass
import math
import struct
import sys

HEADER = struct.Struct('<4sBBHIQfffffHH')
MAGIC = b'AMRL'
VERSION = 1
FLAG_INTENSITY = 0x0001
FLAG_ROS_TIME = 0x0002
MAX_UDP_PAYLOAD = 1472
MODEL_NAMES = {1: 'YDLIDAR X2', 2: 'Camsense X1'}


class PacketError(ValueError):
    """Gói UDP không đúng định dạng AMRL v1."""


@dataclass(frozen=True)
class ScanPacket:
    """Một vòng quét 360 độ đã giải mã."""

    model: int
    flags: int
    seq: int
    stamp_ns: int
    scan_time: float
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    num_valid: int
    ranges_mm: array
    intensities: bytes | None

    @property
    def ros_time(self) -> bool:
        """Stamp là ROS time (ESP32 đã đồng bộ đồng hồ với micro-ROS agent)."""
        return bool(self.flags & FLAG_ROS_TIME)

    @property
    def model_name(self) -> str:
        """Tên model LiDAR."""
        return MODEL_NAMES.get(self.model, f'unknown({self.model})')

    @property
    def num_ranges(self) -> int:
        """Số tia trong vòng quét."""
        return len(self.ranges_mm)

    @property
    def angle_max(self) -> float:
        """Góc của tia cuối cùng (rad), theo quy ước sensor_msgs/LaserScan."""
        return self.angle_min + (len(self.ranges_mm) - 1) * self.angle_increment

    def ranges_m(self) -> list[float]:
        """Khoảng cách (m); tia không có phản hồi trả về +inf (REP-117)."""
        return [r * 0.001 if r else math.inf for r in self.ranges_mm]


def decode(buf: bytes) -> ScanPacket:
    """Giải mã một datagram; ném PacketError nếu không hợp lệ."""
    if len(buf) < HEADER.size:
        raise PacketError(f'gói quá ngắn ({len(buf)} byte)')
    (magic, version, model, flags, seq, stamp_ns, scan_time, angle_min, angle_increment,
     range_min, range_max, num_ranges, num_valid) = HEADER.unpack_from(buf)
    if magic != MAGIC:
        raise PacketError(f'sai magic {magic!r}')
    if version != VERSION:
        raise PacketError(f'version {version} không được hỗ trợ (cần {VERSION})')
    has_intensity = bool(flags & FLAG_INTENSITY)
    expected = HEADER.size + 2 * num_ranges + (num_ranges if has_intensity else 0)
    if len(buf) != expected:
        raise PacketError(f'độ dài {len(buf)} byte, cần {expected} byte cho {num_ranges} tia')
    if num_ranges == 0 or not (angle_increment > 0.0) or not math.isfinite(angle_min):
        raise PacketError('thông tin góc không hợp lệ')

    end = HEADER.size + 2 * num_ranges
    ranges = array('H')
    ranges.frombytes(buf[HEADER.size:end])
    if sys.byteorder != 'little':
        ranges.byteswap()
    intensities = bytes(buf[end:end + num_ranges]) if has_intensity else None
    return ScanPacket(model, flags, seq, stamp_ns, scan_time, angle_min, angle_increment,
                      range_min, range_max, num_valid, ranges, intensities)


def encode(ranges_mm, *, seq=0, stamp_ns=0, ros_time=False, model=1, scan_time=0.0,
           range_min=0.1, range_max=8.0, intensities=None, angle_min=0.0) -> bytes:
    """Đóng gói giống firmware (dùng cho test / giả lập ESP32)."""
    n = len(ranges_mm)
    flags = (FLAG_INTENSITY if intensities is not None else 0) | (FLAG_ROS_TIME if ros_time else 0)
    num_valid = sum(1 for r in ranges_mm if r)
    header = HEADER.pack(MAGIC, VERSION, model, flags, seq, stamp_ns, scan_time, angle_min,
                         2.0 * math.pi / n, range_min, range_max, n, num_valid)
    body = struct.pack(f'<{n}H', *ranges_mm)
    if intensities is not None:
        if len(intensities) != n:
            raise ValueError('intensities phải có cùng số phần tử với ranges_mm')
        body += bytes(intensities)
    return header + body
