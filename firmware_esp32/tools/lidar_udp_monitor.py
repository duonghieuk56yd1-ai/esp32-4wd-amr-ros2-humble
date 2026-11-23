#!/usr/bin/env python3
"""Nhận và giải mã gói LiDAR UDP từ firmware_esp32 (chỉ dùng thư viện chuẩn, không cần ROS).

Dùng để kiểm tra nhanh cầu nối LiDAR trên Laptop:

    python3 tools/lidar_udp_monitor.py              # lắng nghe 0.0.0.0:8889
    python3 tools/lidar_udp_monitor.py --count 5    # thoát sau 5 vòng quét

Định dạng gói (little-endian) — xem lib/amr_core/src/amr_scan_packet.h:
    header '<4sBBHIQfffffHH' (44 byte) + uint16 ranges_mm[N] + [uint8 intensities[N]]

Ánh xạ sang sensor_msgs/LaserScan (gợi ý cho node bridge trên Laptop):
    header.stamp     = stamp_ns nếu flags & 0x2 (ROS time từ ESP32), ngược lại giờ nhận gói
    angle_min        = angle_min_rad
    angle_increment  = angle_increment_rad
    angle_max        = angle_min + (N - 1) * angle_increment
    scan_time        = scan_time_s,  time_increment = 0.0
    range_min/max    = range_min_m / range_max_m
    ranges[i]        = ranges_mm[i] / 1000.0, hoặc +inf nếu ranges_mm[i] == 0
    intensities[i]   = intensities[i] (nếu có)
"""
from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time

HEADER = struct.Struct("<4sBBHIQfffffHH")
MAGIC = b"AMRL"
VERSION = 1
FLAG_INTENSITY = 0x1
FLAG_ROS_TIME = 0x2
MODELS = {1: "YDLIDAR X2", 2: "Camsense X1"}


def decode(buf: bytes) -> dict:
    """Giải mã 1 gói; ném ValueError nếu gói không hợp lệ."""
    if len(buf) < HEADER.size:
        raise ValueError(f"goi qua ngan ({len(buf)} byte)")
    (magic, version, model, flags, seq, stamp_ns, scan_time, angle_min, angle_inc,
     range_min, range_max, n, n_valid) = HEADER.unpack_from(buf)
    if magic != MAGIC:
        raise ValueError(f"sai magic {magic!r}")
    if version != VERSION:
        raise ValueError(f"version {version} khong ho tro")
    has_intensity = bool(flags & FLAG_INTENSITY)
    expected = HEADER.size + 2 * n + (n if has_intensity else 0)
    if len(buf) != expected:
        raise ValueError(f"do dai {len(buf)} != {expected}")

    ranges_mm = struct.unpack_from(f"<{n}H", buf, HEADER.size)
    intensities = list(buf[HEADER.size + 2 * n:]) if has_intensity else []
    return {
        "model": MODELS.get(model, f"unknown({model})"),
        "ros_time": bool(flags & FLAG_ROS_TIME),
        "seq": seq,
        "stamp_ns": stamp_ns,
        "scan_time": scan_time,
        "angle_min": angle_min,
        "angle_increment": angle_inc,
        "range_min": range_min,
        "range_max": range_max,
        "num_valid": n_valid,
        "ranges": [r / 1000.0 if r else math.inf for r in ranges_mm],
        "intensities": intensities,
    }


def describe(scan: dict) -> str:
    ranges = scan["ranges"]
    finite = [(r, i) for i, r in enumerate(ranges) if math.isfinite(r)]
    if finite:
        r_min, i_min = min(finite)
        nearest = (f"gan nhat {r_min:.3f} m @ "
                   f"{math.degrees(scan['angle_min'] + i_min * scan['angle_increment']):6.1f} deg")
    else:
        nearest = "khong co phan hoi"
    front = ranges[0]
    stamp = (f"ros={scan['stamp_ns'] / 1e9:.3f}s" if scan["ros_time"]
             else f"uptime={scan['stamp_ns'] / 1e9:.3f}s")
    hz = 1.0 / scan["scan_time"] if scan["scan_time"] > 0 else 0.0
    return (f"#{scan['seq']:<6} {scan['model']:<11} {len(ranges)} tia, {scan['num_valid']:>3} co phan hoi, "
            f"{hz:4.1f} Hz, {stamp}, phia truoc {front:.3f} m, {nearest}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bind", default="0.0.0.0", help="dia chi lang nghe (mac dinh 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8889, help="cong UDP (mac dinh 8889)")
    parser.add_argument("--count", type=int, default=0, help="thoat sau N goi hop le (0 = chay mai)")
    parser.add_argument("--timeout", type=float, default=0.0, help="thoat (ma loi 1) neu khong co goi sau N giay")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    if args.timeout > 0:
        sock.settimeout(args.timeout)
    print(f"Lang nghe UDP {args.bind}:{args.port} ...", flush=True)

    received = 0
    last_seq = None
    lost = 0
    while args.count == 0 or received < args.count:
        try:
            buf, (host, _) = sock.recvfrom(2048)
        except socket.timeout:
            print(f"Khong nhan duoc goi nao trong {args.timeout} s", file=sys.stderr)
            return 1
        try:
            scan = decode(buf)
        except ValueError as exc:
            print(f"[{host}] goi loi: {exc}", file=sys.stderr)
            continue
        if last_seq is not None and scan["seq"] > last_seq + 1:
            lost += scan["seq"] - last_seq - 1
        last_seq = scan["seq"]
        received += 1
        print(f"[{time.strftime('%H:%M:%S')}][{host}] {describe(scan)} | mat goi: {lost}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
