#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import NavSatFix, Imu
from ament_index_python.packages import get_package_share_directory
import math
import os
import threading

class AutoGpsSaver(Node):
    """
    Automatically saves GPS waypoints and IMU heading to a YAML file at 10 Hz.
    Designed for continuous path recording without manual intervention.
    """
    def __init__(self):
        super().__init__('auto_gps_saver')

        # Thread-safe storage for asynchronous sensor callbacks
        self.gps_lock = threading.Lock()
        self.latest_fix = None
        self.latest_imu = None
        self.waypoint_counter = 1

        # File configuration
        try:
            pkg_share = get_package_share_directory('bee_mobile')
            self.yaml_path = os.path.join(pkg_share, 'config', 'waypoints_GPS.yaml')
        except Exception as e:
            self.get_logger().error(f"Failed to find package 'bee_mobile': {e}")
            raise

        # Subscriptions
        self.fix_sub = self.create_subscription(NavSatFix, '/fix', self.fix_callback, 10)
        self.heading_sub = self.create_subscription(Imu, '/heading_imu', self.imu_callback, 10)

        # Timer setup for 10 Hz execution (0.1 seconds)
        self.timer = self.create_timer(4.0, self.save_waypoint_callback)
        
        self.get_logger().info(f"Auto GPS Saver initialized. Target: {self.yaml_path} @ 10Hz")

    def fix_callback(self, msg):
        with self.gps_lock:
            self.latest_fix = msg
    
    def imu_callback(self, msg):
        with self.gps_lock:
            self.latest_imu = msg

    def euler_from_quaternion(self, q):
        """Converts quaternion to euler angles (ZYX order) to extract physical yaw."""
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

    def save_waypoint_callback(self):
        """Timer-triggered callback to write data to disk."""
        # Secure the latest data state
        with self.gps_lock:
            if not self.latest_fix or not self.latest_imu:
                # Silently wait until both sensors publish valid data
                return
            
            lat = self.latest_fix.latitude
            lon = self.latest_fix.longitude
            _, _, yaw = self.euler_from_quaternion(self.latest_imu.orientation)

        # Perform File I/O outside the lock to prevent blocking the subscription threads
        try:
            os.makedirs(os.path.dirname(self.yaml_path), exist_ok=True)

            if not os.path.exists(self.yaml_path):
                with open(self.yaml_path, 'w') as f:
                    f.write("waypoints_GPS:\n\n")

            with open(self.yaml_path, 'rb') as f:
                f.seek(0, os.SEEK_END)
                if f.tell() > 0:
                    f.seek(-1, os.SEEK_END)
                    last_char = f.read(1)
            
            # Ensure proper YAML formatting with newlines
            if last_char != b'\n':
                with open(self.yaml_path, 'a') as f:
                    f.write('\n')

            with open(self.yaml_path, 'a') as f:
                f.write(f"- name      : WP_{self.waypoint_counter}\n")
                f.write(f"  latitude  : {lat:.7f}\n")
                f.write(f"  longitude : {lon:.7f}\n")
                f.write(f"  yaw       : {yaw:.2f}\n")
                f.write(f"  wait_time : 0.0\n\n")

            self.waypoint_counter += 1

        except IOError as e:
            self.get_logger().error(f"YAML Write Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoGpsSaver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("Shutting down Auto GPS Saver.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()