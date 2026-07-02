#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String
import math
import time
import threading
from collections import deque

class TopicMonitor:
    """Handles thread-safe frequency and std_dev calculations to mimic 'ros2 topic hz'."""
    def __init__(self, window_size=50):
        # Cap the window size to prevent memory leaks over time
        self.arrival_times = deque(maxlen=window_size)
        self.lock = threading.Lock()

    def record(self):
        # Uses OS time for arrival rate, independent of ROS header stamps
        with self.lock:
            self.arrival_times.append(time.time())

    def get_stats(self):
        with self.lock:
            n = len(self.arrival_times)
            if n < 2:
                return 0.0, 0.0

            deltas = [self.arrival_times[i] - self.arrival_times[i-1] for i in range(1, n)]
            mean_delta = sum(deltas) / len(deltas)
            
            if mean_delta <= 0:
                return 0.0, 0.0
                
            hz = 1.0 / mean_delta
            variance = sum((d - mean_delta) ** 2 for d in deltas) / len(deltas)
            std_dev = math.sqrt(variance)
            
            return hz, std_dev

class GpsMonitorNode(Node):
    def __init__(self):
        super().__init__('gps_monitor_node')
        
        # State variables protected by locks to prevent race conditions during timer read
        self.data_lock = threading.Lock()
        self.latest_fix = None
        self.latest_imu = None
        self.latest_status = None
        
        # Frequency monitors
        self.fix_hz = TopicMonitor()
        self.imu_hz = TopicMonitor()
        self.status_hz = TopicMonitor()

        # Subscribers
        self.create_subscription(NavSatFix, '/fix', self.fix_callback, 10)
        self.create_subscription(Imu, '/heading_imu', self.imu_callback, 10)
        self.create_subscription(String, '/gps/status', self.status_callback, 10)

        # Display Timer (2 Hz is optimal for human readability)
        self.create_timer(0.5, self.display_dashboard)

    def fix_callback(self, msg):
        self.fix_hz.record()
        with self.data_lock:
            self.latest_fix = msg

    def imu_callback(self, msg):
        self.imu_hz.record()
        with self.data_lock:
            self.latest_imu = msg

    def status_callback(self, msg):
        self.status_hz.record()
        with self.data_lock:
            self.latest_status = msg.data

    def format_coordinates(self, lat, lon):
        lat_dir = 'N' if lat >= 0 else 'S'
        lon_dir = 'E' if lon >= 0 else 'W'
        return f"{abs(lat):.6f}° {lat_dir}", f"{abs(lon):.6f}° {lon_dir}"

    def euler_from_quaternion(self, q):
        # Convert quaternion to euler angles (ZYX order)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
        cosp = math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
        pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def display_dashboard(self):
        # Clear screen ANSI code for a static dashboard effect
        print("\033[H\033[J", end="") 

        with self.data_lock:
            fix = self.latest_fix
            imu = self.latest_imu
            status = self.latest_status

        fix_hz, fix_std = self.fix_hz.get_stats()
        imu_hz, imu_std = self.imu_hz.get_stats()
        stat_hz, stat_std = self.status_hz.get_stats()

        output = [
            "==================================================================",
            "                       GPS & IMU DASHBOARD                        ",
            "=================================================================="
        ]

        # 1. FIX Data
        if fix:
            stamp = fix.header.stamp
            lat_str, lon_str = self.format_coordinates(fix.latitude, fix.longitude)
            output.append(f"[FIX]   Stamp  : {stamp.sec}.{stamp.nanosec:09d}")
            output.append(f"        Coords : Lat {lat_str} | Lon {lon_str} | Alt {fix.altitude:.2f}m")
        else:
            output.append("[FIX]   En attente de données...")
        output.append(f"        Topic  : {fix_hz:>5.2f} Hz (std_dev: {fix_std:.4f}s)")
        output.append("------------------------------------------------------------------")

        # 2. IMU Data
        if imu:
            r, p, y = self.euler_from_quaternion(imu.orientation)
            output.append(f"[IMU]   Orient : Roll {r:>6.2f}° | Pitch {p:>6.2f}° | Yaw {y:>6.2f}°")
        else:
            output.append("[IMU]   En attente de données...")
        output.append(f"        Topic  : {imu_hz:>5.2f} Hz (std_dev: {imu_std:.4f}s)")
        output.append("------------------------------------------------------------------")

        # 3. STATUS Data
        if status:
            output.append(f"[STAT]  Infos  : {status}")
        else:
            output.append("[STAT]  En attente de données...")
        output.append(f"        Topic  : {stat_hz:>5.2f} Hz (std_dev: {stat_std:.4f}s)")
        output.append("==================================================================")

        # Print the entire block atomically
        print("\n".join(output))

def main(args=None):
    rclpy.init(args=args)
    node = GpsMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print(f"\n[INFO] [{node.get_name()}]: Shutdown requested by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()