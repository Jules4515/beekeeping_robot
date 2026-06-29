#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped
import serial
import math

class CustomGpsDriver(Node):
    def __init__(self):
        super().__init__('custom_gps_driver')
        
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        
        port_name = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud').value
        
        self.get_logger().info(f"[INIT] Target configuration: port={port_name}, baud={baud_rate}")
        
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.heading_pub = self.create_publisher(QuaternionStamped, '/heading', 10)
        
        try:
            # Open port with a short timeout to prevent complete blocking
            self.serial_port = serial.Serial(port_name, baud_rate, timeout=0.5)
            self.get_logger().info(f"[SUCCESS] Serial port {port_name} is open and ready.")
        except serial.SerialException as e:
            self.get_logger().error(f"[CRITICAL] IOError. Cannot open {port_name}: {e}")
            return
            
        # High-frequency timer loop (20 Hz)
        self.timer = self.create_timer(0.01, self.read_serial_data)
        self.get_logger().info("[INIT] Polling timer loop started at 20 Hz.")

    def read_serial_data(self):
        """Polls raw data, handles dynamic buffering, and isolates lines."""
        if not hasattr(self, 'serial_port') or not self.serial_port.is_open:
            self.get_logger().warn("[LOOP] Serial port object missing or closed.")
            return
            
        try:
            bytes_waiting = self.serial_port.in_waiting
            if bytes_waiting > 0:
                self.get_logger().info(f"[STREAM] Raw buffer contains {bytes_waiting} bytes ready to read.")
                
                # Read all available bytes chunks from the OS buffer
                raw_data = self.serial_port.read(bytes_waiting)
                decoded_data = raw_data.decode('ascii', errors='ignore')
                
                # Split using carriage returns or line feeds to support all NMEA formators
                lines = decoded_data.replace('\r', '\n').split('\n')
                
                for line in lines:
                    line = line.strip()
                    if not line:
                        continue
                        
                    # On ignore le "Talker ID" (GN, GP, GL) et on cible la famille de trame
                    if 'GGA,' in line:
                        self.parse_gga(line)
                    elif 'HPR,' in line:
                        self.parse_hpr(line)

                    # Log every single incoming line to trace unknown sentences
                    self.get_logger().info(f"[RAW IN] {line}")

        except Exception as e:
            self.get_logger().error(f"[ERROR] Exception caught in main loop: {e}")

    def parse_gga(self, line):
        """Parses GGA lines and tracks extraction failures."""
        parts = line.split(',')
        self.get_logger().info(f"[PARSING GGA] Segments count: {len(parts)}")
        
        if len(parts) < 10:
            self.get_logger().warn("[PARSE GGA] Sentence is truncated.")
            return
        if not parts[2] or not parts[4]:
            self.get_logger().warn("[PARSE GGA] Lat/Lon fields are empty in this sentence.")
            return
            
        try:
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps_link'
            
            status = int(parts[6])
            msg.status.status = -1 if status == 0 else 0
                
            lat_deg = float(parts[2][:2])
            lat_min = float(parts[2][2:])
            msg.latitude = lat_deg + (lat_min / 60.0)
            if parts[3] == 'S': msg.latitude *= -1.0
            
            lon_deg = float(parts[4][:3])
            lon_min = float(parts[4][3:])
            msg.longitude = lon_deg + (lon_min / 60.0)
            if parts[5] == 'W': msg.longitude *= -1.0
            
            msg.altitude = float(parts[9])
            
            self.fix_pub.publish(msg)
            self.get_logger().info(f"[PUB GGA] Position sent successfully: Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}")
        except Exception as e:
            self.get_logger().error(f"[FAIL GGA] Parsing crashed: {e}")

    def parse_hpr(self, line):
        """Parses HPR lines and tracks extraction failures."""
        parts = line.split(',')
        self.get_logger().info(f"[PARSING HPR] Segments count: {len(parts)}")
        
        if len(parts) < 4:
            self.get_logger().warn("[PARSE HPR] Sentence is truncated.")
            return
        if not parts[2]:
            self.get_logger().warn("[PARSE HPR] Heading field (index 2) is empty.")
            return
            
        try:
            heading_deg = float(parts[2])
            yaw_rad = math.radians(heading_deg)
            
            msg = QuaternionStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps_link'
            
            msg.quaternion.x = 0.0
            msg.quaternion.y = 0.0
            msg.quaternion.z = math.sin(yaw_rad / 2.0)
            msg.quaternion.w = math.cos(yaw_rad / 2.0)
            
            self.heading_pub.publish(msg)
            self.get_logger().info(f"[PUB HPR] SUCCESS. Heading published: {heading_deg:.2f}°")
        except Exception as e:
            self.get_logger().error(f"[FAIL HPR] Parsing crashed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CustomGpsDriver()
    if hasattr(node, 'serial_port') and node.serial_port.is_open:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()