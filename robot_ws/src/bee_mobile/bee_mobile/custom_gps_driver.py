#!/usr/bin/env python3

#
# Custom GPS Driver node for ROS2
#
# This node reads GPS data from a specific serial port, parses the NMEA sentences, and publishes the GPS position, 
# status and heading on specific ROS2 topics. 
#

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String
import serial
import math
import threading
import time

class CustomGpsDriver(Node):
    def __init__(self):
        super().__init__('custom_gps_driver')

        # ROS 2 serial port parameters
        self.declare_parameter('port', '/dev/ttyUSB_GPS')
        self.declare_parameter('baud', 115200)

        self.port_name = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud').value

        # GPS position publisher
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)

        # GPS heading publisher
        self.heading_pub = self.create_publisher(Imu, '/heading_imu', 10)

        # GPS status publisher
        self.status_pub = self.create_publisher(String, '/gps/status', 10)

        self.serial_port = None

        self.current_status = -1
        self.status_start_time = time.time()

        # Read serial data in a dedicated thread
        self.read_thread = threading.Thread(target=self.serial_thread_loop, daemon=True)
        self.read_thread.start()

    def connect_serial(self):
        """Connect or reconnect the serial port without stopping the node"""
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.serial_port = serial.Serial(self.port_name, self.baud_rate, timeout=1.0)
            self.get_logger().info(f"✅ Serial port {self.port_name} connected at {self.baud_rate} baud")
            return True
        except serial.SerialException as e:
            self.get_logger().warn(f"⏳ Waiting for serial port {self.port_name}: {e}")
            return False

    def validate_checksum(self, line):
        """Validate the sentence checksum using XOR"""
        if '*' not in line or not line.startswith('$'):
            return False
        try:
            content, checksum = line[1:].rsplit('*', 1)
            calculated = 0
            for char in content:
                calculated ^= ord(char)
            return f"{calculated:02X}" == checksum.strip().upper()
        except Exception:
            return False

    def serial_thread_loop(self):
        """Read and dispatch GPS sentences in the serial thread"""
        while rclpy.ok():
            if self.serial_port is None or not self.serial_port.is_open:
                if not self.connect_serial():
                    time.sleep(2.0)
                    continue
            
            try:
                # Read one complete sentence from the serial port
                line = self.serial_port.readline()
                
                if line:
                    reception_time = self.get_clock().now()
                    decoded_line = line.decode('ascii', errors='ignore').strip()
                    
                    if not decoded_line:
                        continue
                        
                    # Ignore sentences with an invalid checksum
                    if not self.validate_checksum(decoded_line):
                        continue
                    
                    # Dispatch supported sentence types
                    if 'GGA,' in decoded_line:
                        self.parse_gga(decoded_line, reception_time)
                    elif 'HPR,' in decoded_line:
                        self.parse_hpr(decoded_line, reception_time)
                        
            except serial.SerialException as e:
                self.get_logger().error(f"❌ Serial connection lost: {e}")
                self.serial_port.close()
                self.serial_port = None
            except Exception as e:
                self.get_logger().error(f"Serial thread error: {e}")

    def parse_gga(self, line, reception_time):
        """Parse a GGA sentence and publish the GPS position"""
        parts = line.split(',')
        if len(parts) < 10 or not parts[2] or not parts[4]:
            return
            
        try:
            msg = NavSatFix()
            msg.header.stamp = reception_time.to_msg()
            msg.header.frame_id = 'gps_link'
            
            status = int(parts[6]) if parts[6] else 0
            msg.status.status = status
            msg.status.service = 1
            
            lat_deg = float(parts[2][:2])
            lat_min = float(parts[2][2:])
            msg.latitude = lat_deg + (lat_min / 60.0)
            if parts[3] == 'S': msg.latitude *= -1.0
            
            lon_deg = float(parts[4][:3])
            lon_min = float(parts[4][3:])
            msg.longitude = lon_deg + (lon_min / 60.0)
            if parts[5] == 'W': msg.longitude *= -1.0
            
            msg.altitude = float(parts[9])
            
            # Set covariance values based on the GPS fix quality
            if status == 4:     # RTK Fixed (~2cm precision)
                var_h, var_v = 0.0004, 0.0016
                status_str = 'RTK_FIXED'
            elif status == 5:   # RTK Float (~20cm)
                var_h, var_v = 0.04, 0.16
                status_str = 'RTK_FLOAT'
            elif status == 2:   # DGPS (~50cm)
                var_h, var_v = 0.25, 1.0
                status_str = 'DGPS'
            else:               # No Fix / GPS Standard (>2m)
                var_h, var_v = 4.0, 16.0
                status_str = 'GPS_POOR' if status == 1 else 'NO_FIX'
                
            msg.position_covariance[0] = var_h
            msg.position_covariance[4] = var_h
            msg.position_covariance[8] = var_v
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            
            self.fix_pub.publish(msg)
            
            # Extract the number of satellites from the GGA sentence (index 7)
            satellites = int(parts[7]) if len(parts) > 7 and parts[7] else 0
            
            # Track how long the current status has been active (used in gps_monitor node)
            current_time = time.time()
            if self.current_status != status:
                self.current_status = status
                self.status_start_time = current_time
            
            duration = int(current_time - self.status_start_time)
            
            # Publish the current GPS status
            status_msg = String()
            status_msg.data = f"{status} {status_str} | {satellites} satellites | For {duration}s"
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"GGA parsing error: {e} | Sentence: {line}")

    def parse_hpr(self, line, reception_time):
        """Parse an HPR sentence and publish the GPS heading"""
        parts = line.split(',')
        if len(parts) < 4 or not parts[2]:
            return
            
        try:
            heading_deg = float(parts[2])
            pitch_deg = float(parts[3]) if len(parts) > 3 and parts[3] else 0.0
            
            # Extract roll and remove an attached checksum when present
            roll_str = parts[4].split('*')[0] if len(parts) > 4 and parts[4] else "0.0"
            roll_deg = float(roll_str)
            
            # Convert heading from NED to ROS ENU coordinates
            heading_rad = math.radians(heading_deg)
            yaw_rad = math.pi / 2.0 - heading_rad
            
            # Normalize yaw to the range [-pi, pi]
            yaw_rad = (yaw_rad + math.pi) % (2 * math.pi) - math.pi
            
            pitch_rad = math.radians(pitch_deg)
            roll_rad = math.radians(roll_deg)
            
            # Convert Euler angles to a quaternion using ZYX order
            cy = math.cos(yaw_rad * 0.5)
            sy = math.sin(yaw_rad * 0.5)
            cp = math.cos(pitch_rad * 0.5)
            sp = math.sin(pitch_rad * 0.5)
            cr = math.cos(roll_rad * 0.5)
            sr = math.sin(roll_rad * 0.5)
            
            msg = Imu()
            msg.header.stamp = reception_time.to_msg()
            msg.header.frame_id = 'gps_link'
            
            msg.orientation.w = cr * cp * cy + sr * sp * sy
            msg.orientation.x = sr * cp * cy - cr * sp * sy
            msg.orientation.y = cr * sp * cy + sr * cp * sy
            msg.orientation.z = cr * cp * sy - sr * sp * cy
            
            # Set the covariance for the dual antenna heading
            msg.orientation_covariance[0] = 0.001
            msg.orientation_covariance[4] = 0.001
            msg.orientation_covariance[8] = 0.001
            
            self.heading_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"HPR parsing error: {e} | Sentence: {line}")


def main(args=None):
    rclpy.init(args=args)
    node = CustomGpsDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print(f"[INFO] [{node.get_name()}]: Shutdown requested by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()