#!/usr/bin/env python3
# 
# GPS Trajectory Recorder for ROS2
#
# This node records GPS and IMU poses into timestamped YAML trajectory files.
# It saves an initial waypoint, adds spatial waypoints when distance or heading
# thresholds are exceeded, and supports manually recorded TASK waypoints from
# the joystick controller.
# 

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Empty
import math
import os
import threading
from datetime import datetime

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        # Dynamic ROS 2 parameters
        self.declare_parameter('delta_d', 3.0)
        self.declare_parameter('delta_theta', 50.0)
        
        self.delta_d = self.get_parameter('delta_d').value
        self.delta_theta = self.get_parameter('delta_theta').value

        # Thread-safe storage for asynchronous sensor callbacks
        self.gps_lock = threading.Lock()
        self.latest_fix = None
        self.latest_imu = None
        
        # Internal in-memory state.
        self.last_saved_lat = None
        self.last_saved_lon = None
        self.last_saved_alt = None
        self.last_saved_roll = None
        self.last_saved_pitch = None
        self.last_saved_yaw = None
        self.waypoint_counter = 1
        self.task_waypoint_counter = 1
        
        # Listen for the joystick signal.
        self.task_sub = self.create_subscription(Empty, '/save_task_waypoint', self.task_callback, 10)

        # Configure the output file.
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"trajectory_{timestamp}.yaml"
        
        # Use the workspace source directory for access outside the build tree.
        ws_path = os.path.expanduser('~/dev/robot_ws/src/bee_mobile/trajectories')
        os.makedirs(ws_path, exist_ok=True)
        self.yaml_path = os.path.join(ws_path, filename)

        # Initialize the YAML file.
        with open(self.yaml_path, 'w') as f:
            f.write("waypoints_GPS:\n\n")

        # Subscriptions
        self.fix_sub = self.create_subscription(NavSatFix, '/fix', self.fix_callback, 10)
        self.heading_sub = self.create_subscription(Imu, '/heading_imu', self.imu_callback, 10)

        # Process the trajectory continuously at 20 Hz.
        self.timer = self.create_timer(0.05, self.process_trajectory)
        
        self.get_logger().info("=== SPATIAL TRAJECTORY RECORDER INITIALIZED ===")
        self.get_logger().info(f"Parameters: Delta D = {self.delta_d} m | Delta Theta = {self.delta_theta}°")

    def fix_callback(self, msg):
        with self.gps_lock:
            self.latest_fix = msg
    
    def imu_callback(self, msg):
        with self.gps_lock:
            self.latest_imu = msg

    @staticmethod
    def euler_from_quaternion(q):
        """Converts quaternion to euler angles (ZYX order) in degrees."""
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

    @staticmethod
    def calculate_haversine_distance(lat1, lon1, lat2, lon2):
        """Calculate the absolute distance in meters between two GPS points."""
        R = 6371000.0  # Earth radius in meters.
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        
        a = math.sin(dphi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0)**2
        return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))

    @staticmethod
    def calculate_angular_difference(yaw1, yaw2):
        """Calculate the minimum angular difference across the -180/180 boundary."""
        diff = yaw1 - yaw2
        return (diff + 180) % 360 - 180

    def process_trajectory(self):
        """Run the high-frequency spatial filtering logic."""
        with self.gps_lock:
            if not self.latest_fix or not self.latest_imu:
                return
            
            current_lat = self.latest_fix.latitude
            current_lon = self.latest_fix.longitude
            current_alt = self.latest_fix.altitude
            
            # Extract the complete 6-DoF orientation.
            current_roll, current_pitch, current_yaw = self.euler_from_quaternion(self.latest_imu.orientation)

        # 1. Unconditionally save the first point (origin).
        if self.last_saved_lat is None:
            self.save_to_disk(current_lat, current_lon, current_alt, current_roll, current_pitch, current_yaw)
            return

        # 2. Calculate the physical deltas.
        dist = self.calculate_haversine_distance(self.last_saved_lat, self.last_saved_lon, current_lat, current_lon)
        angle_diff = abs(self.calculate_angular_difference(self.last_saved_yaw, current_yaw))

        # 3. Conditional evaluation (spatial filter).
        if dist >= self.delta_d or angle_diff >= self.delta_theta:
            self.save_to_disk(current_lat, current_lon, current_alt, current_roll, current_pitch, current_yaw)

    def save_to_disk(self, lat, lon, alt, roll, pitch, yaw, custom_name=None):
        """Write the 6-DoF pose to disk."""
        wp_name = custom_name if custom_name else f"WP_{self.waypoint_counter}"
        
        try:
            with open(self.yaml_path, 'a') as f:
                f.write(f"- name      : {wp_name}\n")
                f.write(f"  latitude  : {lat:.7f}\n")
                f.write(f"  longitude : {lon:.7f}\n")
                f.write(f"  altitude  : {alt:.2f}\n")
                f.write(f"  roll      : {roll:.2f}\n")
                f.write(f"  pitch     : {pitch:.2f}\n")
                f.write(f"  yaw       : {yaw:.2f}\n\n")

            # Update the in-memory cache for the standard spatial trajectory only.
            if not custom_name:
                self.last_saved_lat = lat
                self.last_saved_lon = lon
                self.last_saved_alt = alt
                self.last_saved_roll = roll
                self.last_saved_pitch = pitch
                self.last_saved_yaw = yaw
                self.waypoint_counter += 1
                self.get_logger().info(f"[{wp_name}] Saved | Distance: {self.delta_d} m or heading: {self.delta_theta}° threshold reached.")

        except IOError as e:
            self.get_logger().error(f"YAML Write Error: {e}")

    def task_callback(self, msg):
        """Handle the joystick request to save a specific TASK waypoint."""
        with self.gps_lock:
            if not self.latest_fix or not self.latest_imu:
                self.get_logger().warn("Unable to save WP_TASK: GPS or IMU data is missing.")
                return
            
            lat = self.latest_fix.latitude
            lon = self.latest_fix.longitude
            alt = self.latest_fix.altitude
            roll, pitch, yaw = self.euler_from_quaternion(self.latest_imu.orientation)
            
        wp_name = f"WP_TASK_{self.task_waypoint_counter}"
        self.save_to_disk(lat, lon, alt, roll, pitch, yaw, custom_name=wp_name)
        
        self.get_logger().info(f"[{wp_name}] Manual waypoint saved.")
        self.task_waypoint_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Use print() strictly to avoid a /rosout context crash.
        print(f"[INFO] [trajectory_recorder]: Trajectory saved to {node.yaml_path}")
        print("[INFO] [trajectory_recorder]: Stopping")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()