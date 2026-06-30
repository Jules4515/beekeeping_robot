#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped
import math

class GPSHeadingDisplay(Node):
    def __init__(self):
        super().__init__('gps_heading_display')
        
        # Subscriptions
        self.fix_sub = self.create_subscription(
            NavSatFix,
            '/fix',
            self.fix_callback,
            10
        )
        self.heading_sub = self.create_subscription(
            QuaternionStamped,  # Changed from Float64 to QuaternionStamped
            '/heading',
            self.heading_callback,
            10
        )
        
        # Store latest data
        self.latest_fix = None
        self.latest_heading_quat = None
        
        # Timer to print periodically (2 Hz)
        self.timer = self.create_timer(0.5, self.print_status)
        
        self.get_logger().info("GPS Heading Display Node Started")
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle in degrees"""
        # yaw = atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw_rad)
    
    def fix_callback(self, msg):
        self.latest_fix = msg
    
    def heading_callback(self, msg):
        self.latest_heading_quat = msg.quaternion
    
    def print_status(self):
        if self.latest_fix is None:
            self.get_logger().warn("Waiting for /fix message...")
            return
        
        lat = self.latest_fix.latitude
        lon = self.latest_fix.longitude
        alt = self.latest_fix.altitude
        
        if self.latest_heading_quat is not None:
            heading_deg = self.quaternion_to_yaw(self.latest_heading_quat)
            heading_str = f"Heading: {heading_deg:.1f}°"
        else:
            heading_str = "Heading: not available"
        
        self.get_logger().info(
            f"Position: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}m | {heading_str}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = GPSHeadingDisplay()
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