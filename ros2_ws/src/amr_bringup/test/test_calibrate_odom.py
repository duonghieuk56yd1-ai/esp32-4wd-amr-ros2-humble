"""Toán hiệu chỉnh odometry của scripts/calibrate_odom.py (không cần robot)."""

import importlib.util
import math
import os

import pytest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(PKG_DIR, 'scripts', 'calibrate_odom.py')
XACRO = os.path.join(os.path.dirname(PKG_DIR), 'amr_description', 'urdf', 'amr_3wd.urdf.xacro')


@pytest.fixture(scope='module')
def cal():
    spec = importlib.util.spec_from_file_location('calibrate_odom', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def simulate_odom(cal, left_arc, right_arc, separation, steps=600):
    """Tích phân odometry giống firmware từ quãng đường lăn của 2 bánh, rồi đưa qua tracker."""
    tracker = cal.OdomTracker()
    x = y = yaw = 0.0
    tracker.add(x, y, cal.wrap_angle(yaw))
    for _ in range(steps):
        dl, dr = left_arc / steps, right_arc / steps
        ds, dth = 0.5 * (dl + dr), (dr - dl) / separation
        x += ds * math.cos(yaw + 0.5 * dth)
        y += ds * math.sin(yaw + 0.5 * dth)
        yaw += dth
        tracker.add(x, y, cal.wrap_angle(yaw))  # /odom chỉ chứa yaw đã quấn về ±180°
    return tracker


def test_angle_helpers(cal):
    assert cal.wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert cal.wrap_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    q = math.pi / 4  # quaternion quay 90° quanh z: (z, w) = (sin 45°, cos 45°)
    assert cal.yaw_from_quaternion(math.sin(q), math.cos(q)) == pytest.approx(math.pi / 2)


def test_tracker_straight_line(cal):
    tracker = cal.OdomTracker()
    assert tracker.distance == 0.0
    for i in range(101):
        tracker.add(1.0 + 0.6 * i / 100, 2.0 + 0.8 * i / 100, 0.9273)
    assert tracker.distance == pytest.approx(1.0)
    assert tracker.total_yaw == pytest.approx(0.0)


@pytest.mark.parametrize('turns', [3.0, -2.5])
def test_tracker_unwraps_multiple_turns(cal, turns):
    tracker = cal.OdomTracker()
    steps = int(abs(turns) * 36)
    for i in range(steps + 1):
        tracker.add(0.0, 0.0, cal.wrap_angle(math.copysign(1, turns) * math.radians(10) * i))
    assert tracker.turns == pytest.approx(turns)
    assert tracker.distance == 0.0


def test_corrected_wheel_radius(cal):
    assert cal.corrected_wheel_radius(0.0325, 0.97, 1.0) == pytest.approx(0.0325 / 0.97)
    with pytest.raises(ValueError):
        cal.corrected_wheel_radius(0.0325, 0.0, 1.0)


def test_corrected_wheel_separation(cal):
    assert cal.corrected_wheel_separation(0.18, 2 * math.pi * 5.2, 5) == pytest.approx(0.1872)
    # Quay theo chiều kim đồng hồ (yaw âm) cho cùng kết quả
    assert cal.corrected_wheel_separation(0.18, -2 * math.pi * 5.2, 5) == pytest.approx(0.1872)
    with pytest.raises(ValueError):
        cal.corrected_wheel_separation(0.18, 1.0, 0)


def test_calibration_recovers_true_wheel_radius(cal):
    """Firmware dùng r = 0.0325 nhưng bánh thật r = 0.0335: đi thẳng 1 m thật."""
    r_fw, r_true, sep = 0.0325, 0.0335, 0.18
    wheel_angle = 1.0 / r_true                   # bánh quay để xe đi 1 m thật
    arc_fw = wheel_angle * r_fw                  # quãng đường firmware tính ra
    tracker = simulate_odom(cal, arc_fw, arc_fw, sep)
    assert tracker.distance == pytest.approx(r_fw / r_true)
    assert cal.corrected_wheel_radius(r_fw, tracker.distance, 1.0) == pytest.approx(r_true)


def test_calibration_recovers_true_wheel_separation(cal):
    """Firmware dùng L = 0.18 nhưng L hiệu dụng thật = 0.19: quay tại chỗ đúng 5 vòng."""
    l_fw, l_true, turns = 0.18, 0.19, 5
    arc = math.pi * l_true * turns               # mỗi bánh lăn nửa chu vi vòng tròn L thật
    tracker = simulate_odom(cal, -arc, arc, l_fw, steps=3000)
    assert tracker.turns == pytest.approx(turns * l_true / l_fw)
    assert tracker.distance == pytest.approx(0.0, abs=1e-9)
    assert cal.corrected_wheel_separation(l_fw, tracker.total_yaw, turns) == \
        pytest.approx(l_true)


def test_urdf_defaults_match_firmware_defaults(cal):
    values = cal.urdf_wheel_params(XACRO)
    assert values == {'wheel_radius': pytest.approx(0.0325),
                      'wheel_separation': pytest.approx(0.18)}
    assert cal.urdf_wheel_params('/khong/ton/tai.xacro') == {
        'wheel_radius': cal.DEFAULT_WHEEL_RADIUS,
        'wheel_separation': cal.DEFAULT_WHEEL_SEPARATION}


def test_report_prints_new_values(cal, capsys):
    tracker = simulate_odom(cal, 0.97, 0.97, 0.18)
    cal.report(tracker, 0.0325, 0.18, real_distance=1.0, ask=False)
    out = capsys.readouterr().out
    assert 'Quãng đường theo odom : 0.9700 m' in out
    assert '#define WHEEL_RADIUS_M 0.03351f' in out
    assert 'wheel_radius" default="0.03351"' in out
    assert 'WHEEL_SEPARATION_M mới' not in out  # không quay -> không đề xuất L


def test_report_without_odom(cal, capsys):
    cal.report(cal.OdomTracker(), 0.0325, 0.18, ask=False)
    assert 'Chưa nhận đủ /odom' in capsys.readouterr().out
