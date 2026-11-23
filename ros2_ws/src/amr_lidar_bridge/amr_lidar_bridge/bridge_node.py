"""
Node ROS 2: nhận gói LiDAR UDP (cổng 8889) từ ESP32 và xuất bản /scan.

Mỗi datagram là một vòng quét 360 độ đã được firmware chuẩn hoá theo quy ước
ROS (tia i ở góc angle_min + i * angle_increment, ngược chiều kim đồng hồ), nên
node chỉ việc chép sang sensor_msgs/msg/LaserScan với frame_id = laser_frame.
"""

from __future__ import annotations

from array import array
import socket
import threading

from builtin_interfaces.msg import Time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from amr_lidar_bridge.packet import decode, PacketError, ScanPacket

NS_PER_S = 1_000_000_000


def select_stamp_ns(now_ns: int, packet: ScanPacket, use_device_stamp: bool,
                    max_stamp_skew: float, time_offset: float = 0.0):
    """
    Chọn thời điểm (ns) gắn vào LaserScan.

    Dùng stamp ESP32 gửi kèm khi được phép, ESP32 đã đồng bộ đồng hồ với micro-ROS
    agent (packet.ros_time) và stamp không lệch quá max_stamp_skew giây so với giờ
    Laptop; ngược lại dùng giờ nhận gói now_ns. Luôn cộng thêm time_offset (s).

    Trả về (stamp_ns, rejected_skew): rejected_skew là độ lệch (s) khi stamp ESP32
    bị loại vì lệch quá nhiều, ngược lại None.
    """
    stamp_ns = now_ns
    rejected_skew = None
    if use_device_stamp and packet.ros_time:
        skew = (now_ns - packet.stamp_ns) / NS_PER_S
        if abs(skew) <= max_stamp_skew:
            stamp_ns = packet.stamp_ns
        else:
            rejected_skew = skew
    return stamp_ns + int(time_offset * NS_PER_S), rejected_skew


def scan_to_msg(packet: ScanPacket, frame_id: str, stamp_ns: int,
                range_min: float = -1.0, range_max: float = -1.0) -> LaserScan:
    """
    Chép một vòng quét đã giải mã sang sensor_msgs/msg/LaserScan.

    Tia không có phản hồi thành +inf (REP-117). range_min/range_max > 0 ghi đè giá
    trị ESP32 gửi kèm.
    """
    msg = LaserScan()
    msg.header.stamp = Time(sec=stamp_ns // NS_PER_S, nanosec=stamp_ns % NS_PER_S)
    msg.header.frame_id = frame_id
    msg.angle_min = packet.angle_min
    msg.angle_max = packet.angle_max
    msg.angle_increment = packet.angle_increment
    msg.time_increment = 0.0
    msg.scan_time = packet.scan_time
    msg.range_min = range_min if range_min > 0.0 else packet.range_min
    msg.range_max = range_max if range_max > 0.0 else packet.range_max
    msg.ranges = array('f', packet.ranges_m())
    if packet.intensities is not None:
        # list(): array('f', bytes) sẽ đọc bytes như float thô
        msg.intensities = array('f', list(packet.intensities))
    return msg


class LidarUdpBridge(Node):
    """Cầu nối UDP -> sensor_msgs/LaserScan."""

    def __init__(self, **kwargs):
        super().__init__('lidar_bridge', **kwargs)
        self.udp_port = int(self.declare_parameter('udp_port', 8889).value)
        self.bind_address = str(self.declare_parameter('bind_address', '0.0.0.0').value)
        self.frame_id = str(self.declare_parameter('frame_id', 'laser_frame').value)
        # true: dùng stamp ESP32 gửi kèm (đồng hồ đã đồng bộ với micro-ROS agent,
        # nhất quán với /odom và /tf); false: dùng giờ Laptop lúc nhận gói.
        self.use_device_stamp = bool(self.declare_parameter('use_device_stamp', True).value)
        self.max_stamp_skew = float(self.declare_parameter('max_stamp_skew', 1.0).value)
        self.time_offset = float(self.declare_parameter('time_offset', 0.0).value)
        self.range_min = float(self.declare_parameter('range_min', -1.0).value)
        self.range_max = float(self.declare_parameter('range_max', -1.0).value)
        self.allowed_source_ip = str(self.declare_parameter('allowed_source_ip', '').value)
        reliable = bool(self.declare_parameter('reliable_qos', True).value)
        self.data_timeout = float(self.declare_parameter('data_timeout', 2.0).value)

        # Reliable tương thích với mọi subscriber (RViz, slam_toolbox, Nav2 costmap).
        qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(LaserScan, 'scan', qos)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError:
            pass
        try:
            self.sock.bind((self.bind_address, self.udp_port))
        except OSError as exc:
            self.sock.close()
            self.get_logger().fatal(
                f'Không mở được UDP {self.bind_address}:{self.udp_port}: {exc}')
            raise
        self.sock.settimeout(0.25)

        self._lock = threading.Lock()
        self._last_rx_ns = 0
        self._last_seq = None
        self._source = None
        self._received = 0
        self._lost = 0
        self._bad = 0
        self._timed_out = False

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._receive_loop, name='lidar_udp_rx',
                                        daemon=True)
        self._thread.start()
        self.create_timer(1.0, self._check_health)
        self.get_logger().info(
            f'Lắng nghe gói LiDAR UDP {self.bind_address}:{self.udp_port} '
            f'-> {self.publisher.topic_name} (frame_id={self.frame_id})')

    # ------------------------------------------------------------------ nhận gói
    def _receive_loop(self):
        while not self._stop.is_set():
            try:
                data, (host, _) = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break  # socket đã đóng khi tắt node
            if self.allowed_source_ip and host != self.allowed_source_ip:
                continue
            try:
                packet = decode(data)
            except PacketError as exc:
                with self._lock:
                    self._bad += 1
                self.get_logger().warning(f'Gói LiDAR lỗi từ {host}: {exc}',
                                          throttle_duration_sec=5.0)
                continue
            self._publish(packet, host)

    def _publish(self, packet: ScanPacket, host: str):
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns, rejected_skew = select_stamp_ns(
            now_ns, packet, self.use_device_stamp, self.max_stamp_skew, self.time_offset)
        if rejected_skew is not None:
            self.get_logger().warning(
                f'Stamp ESP32 lệch {rejected_skew:+.2f} s so với Laptop (> max_stamp_skew) '
                '-> dùng giờ nhận gói', throttle_duration_sec=10.0)
        self.publisher.publish(
            scan_to_msg(packet, self.frame_id, stamp_ns, self.range_min, self.range_max))

        with self._lock:
            first = self._source is None
            if self._last_seq is not None:
                gap = (packet.seq - self._last_seq) & 0xFFFFFFFF
                if 1 < gap < 1000:
                    self._lost += gap - 1
            self._last_seq = packet.seq
            self._source = host
            self._received += 1
            self._last_rx_ns = now_ns
        if first:
            hz = 1.0 / packet.scan_time if packet.scan_time > 0.0 else 0.0
            self.get_logger().info(
                f'Nhận LiDAR {packet.model_name} từ {host}: {packet.num_ranges} tia, '
                f'{hz:.1f} Hz, stamp {"ROS time từ ESP32" if packet.ros_time else "giờ Laptop"}')

    # ----------------------------------------------------------------- giám sát
    def _check_health(self):
        with self._lock:
            last_rx_ns, received, lost, bad = (self._last_rx_ns, self._received, self._lost,
                                               self._bad)
        if received == 0:
            self.get_logger().warning(
                f'Chưa nhận được gói LiDAR nào trên UDP {self.udp_port} (gói lỗi: {bad}) - '
                f'kiểm tra ESP32, Wi-Fi, firewall (sudo ufw allow {self.udp_port}/udp)',
                throttle_duration_sec=10.0)
            return
        silent = (self.get_clock().now().nanoseconds - last_rx_ns) / NS_PER_S
        if silent > self.data_timeout:
            if not self._timed_out:
                self._timed_out = True
                self.get_logger().warning(
                    f'Mất dữ liệu LiDAR {silent:.1f} s (đã nhận {received} vòng, mất {lost}, '
                    f'lỗi {bad})')
        elif self._timed_out:
            self._timed_out = False
            self.get_logger().info('Đã nhận lại dữ liệu LiDAR')

    def destroy_node(self):
        """Dừng luồng nhận và đóng socket."""
        self._stop.set()
        try:
            self.sock.close()
        except OSError:
            pass
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarUdpBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
