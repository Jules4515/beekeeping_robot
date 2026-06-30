#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped, Point
from visualization_msgs.msg import Marker
import math
import utm

class RobotFootprintPublisher(Node):
    def __init__(self):
        super().__init__('robot_footprint_publisher')
        
        # Robot dimensions (meters)
        self.length = 1.6
        self.width = 1.2
        
        # Store current data
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_position = False
        self.has_heading = False
        
        # Reference point for local coordinates
        self.ref_lat = None
        self.ref_lon = None
        self.ref_set = False
        self.earth_radius = 6371000.0
        
        # Subscriptions
        self.heading_sub = self.create_subscription(
            QuaternionStamped,
            '/heading',
            self.heading_callback,
            10
        )
        
        self.fix_sub = self.create_subscription(
            NavSatFix,
            '/fix',
            self.fix_callback,
            10
        )
        
        # Publisher for robot footprint marker
        self.marker_pub = self.create_publisher(
            Marker,
            '/robot_footprint',
            10
        )
        
        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_footprint)
        
        self.get_logger().info("Robot Footprint Publisher Started")

    
    def latlon_to_xy(self, lat, lon):
        """Convert lat/lon to UTM coordinates"""
        easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)
        
        if not self.ref_set:
            self.ref_x = easting
            self.ref_y = northing
            self.ref_set = True
            return 0.0, 0.0
    
        return easting - self.ref_x, northing - self.ref_y
    
    def heading_callback(self, msg):
        """Extract yaw from quaternion heading"""
        q = msg.quaternion
        # Convert quaternion to yaw angle
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_heading = True
    
    def fix_callback(self, msg):
        """Convert GPS to local coordinates"""
        if msg.latitude != 0.0 and msg.longitude != 0.0:
            self.current_x, self.current_y = self.latlon_to_xy(
                msg.latitude, msg.longitude
            )
            self.has_position = True
    
    def publish_footprint(self):
        """Publish robot footprint as LINE_STRIP marker"""
        if not self.has_position or not self.has_heading:
            return
        
        # Calculate rectangle corners in robot frame
        half_length = self.length / 2.0
        half_width = self.width / 2.0
        
        corners_robot = [
            ( half_length,  half_width),  # Front-right
            ( half_length, -half_width),  # Front-left
            (-half_length, -half_width),  # Rear-left
            (-half_length,  half_width),  # Rear-right
            ( half_length,  half_width)   # Back to start
        ]
        
        # Transform to map frame
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        
        points = []
        for x_robot, y_robot in corners_robot:
            x_map = self.current_x + (x_robot * cos_yaw - y_robot * sin_yaw)
            y_map = self.current_y + (x_robot * sin_yaw + y_robot * cos_yaw)
            
            point = Point()
            point.x = x_map
            point.y = y_map
            point.z = 0.0
            points.append(point)
        
        # Create LINE_STRIP marker (outline)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_footprint"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08  # Line thickness
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque
        marker.points = points
        
        # Create filled polygon (semi-transparent)
        polygon_marker = Marker()
        polygon_marker.header.frame_id = "map"
        polygon_marker.header.stamp = self.get_clock().now().to_msg()
        polygon_marker.ns = "robot_footprint_filled"
        polygon_marker.id = 1
        polygon_marker.type = Marker.TRIANGLE_LIST
        polygon_marker.action = Marker.ADD
        polygon_marker.scale.x = 1.0
        polygon_marker.color.r = 0.0
        polygon_marker.color.g = 1.0
        polygon_marker.color.b = 0.0
        polygon_marker.color.a = 0.3  # Semi-transparent fill
        
        # Triangulate the rectangle (two triangles)
        if len(points) >= 4:
            polygon_marker.points.append(points[0])
            polygon_marker.points.append(points[1])
            polygon_marker.points.append(points[2])
            
            polygon_marker.points.append(points[0])
            polygon_marker.points.append(points[2])
            polygon_marker.points.append(points[3])
        
        # Publish both markers
        self.marker_pub.publish(marker)
        self.marker_pub.publish(polygon_marker)

def main(args=None):
    rclpy.init(args=args)
    node = RobotFootprintPublisher()
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