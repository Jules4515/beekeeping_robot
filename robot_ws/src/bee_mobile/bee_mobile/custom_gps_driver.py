#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import QuaternionStamped
from std_msgs.msg import String
import serial
import math

class CustomGpsDriver(Node):
    def __init__(self):
        super().__init__('custom_gps_driver')
        
        self.declare_parameter('port', '/dev/ttyUSB_GPS')
        self.declare_parameter('baud', 115200)
        
        port_name = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud').value
        
        self.get_logger().info(f"Ouverture de {port_name} à {baud_rate} bauds")
        
        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.heading_pub = self.create_publisher(Imu, '/heading_imu', 10)
        self.status_pub = self.create_publisher(String, '/gps/status', 10)
        
        try:
            self.serial_port = serial.Serial(port_name, baud_rate, timeout=0.5)
            self.get_logger().info(f"Port {port_name} ouvert")
        except serial.SerialException as e:
            self.get_logger().error(f"Erreur port série: {e}")
            return
        
        self.timer = self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        if not hasattr(self, 'serial_port') or not self.serial_port.is_open:
            return
            
        try:
            bytes_waiting = self.serial_port.in_waiting
            if bytes_waiting > 0:
                raw_data = self.serial_port.read(bytes_waiting)
                decoded_data = raw_data.decode('ascii', errors='ignore')
                lines = decoded_data.replace('\r', '\n').split('\n')
                
                for line in lines:
                    line = line.strip()
                    if not line:
                        continue
                    
                    if 'GGA,' in line:
                        self.parse_gga(line)
                    elif 'HPR,' in line:
                        self.parse_hpr(line)
                        
        except Exception as e:
            self.get_logger().error(f"Erreur lecture: {e}")

    def parse_gga(self, line):
        parts = line.split(',')
        
        if len(parts) < 10:
            return
        if not parts[2] or not parts[4]:
            return
        
        try:
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps_link'
            
            # ✅ CORRECTION : Garder le VRAI status GPS
            status = int(parts[6]) if parts[6] else 0
            msg.status.status = status
            msg.status.service = 1  # GPS
            
            lat_deg = float(parts[2][:2])
            lat_min = float(parts[2][2:])
            msg.latitude = lat_deg + (lat_min / 60.0)
            if parts[3] == 'S':
                msg.latitude *= -1.0
            
            lon_deg = float(parts[4][:3])
            lon_min = float(parts[4][3:])
            msg.longitude = lon_deg + (lon_min / 60.0)
            if parts[5] == 'W':
                msg.longitude *= -1.0
            
            msg.altitude = float(parts[9])
            
            # Covariance typique RTK : ~2cm en horizontal
            if status >= 4:
                msg.position_covariance[0] = 0.0004   # 2cm
                msg.position_covariance[4] = 0.0004
                msg.position_covariance[8] = 0.0016   # 4cm vertical
            else:
                msg.position_covariance[0] = 1.0
                msg.position_covariance[4] = 1.0
                msg.position_covariance[8] = 4.0
            
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            
            # Publier le statut GPS
            status_msg = String()
            if status >= 4:
                status_msg.data = 'RTK_FIXED'
            elif status == 2:
                status_msg.data = 'DGPS'
            elif status == 1:
                status_msg.data = 'GPS'
            else:
                status_msg.data = 'NO_FIX'
            self.status_pub.publish(status_msg)

            self.fix_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Erreur parsing GGA: {e}")

    def parse_hpr(self, line):
        parts = line.split(',')
        
        if len(parts) < 4:
            return
        if not parts[2]:
            return
        
        try:
            heading_deg = float(parts[2])
            pitch_deg = float(parts[3]) if len(parts) > 3 and parts[3] else 0.0
            roll_deg = float(parts[4]) if len(parts) > 4 and parts[4] else 0.0
            
            # ✅ CORRECTION : NED → ENU
            # NED : 0° = Nord, sens horaire
            # ENU (ROS) : 0° = Est, sens anti-horaire
            # Formule : yaw_ros = 90° - heading_ned (en radians : pi/2 - heading_rad)
            heading_rad = math.radians(heading_deg)
            yaw_rad = math.pi/2 - heading_rad
            
            # Normaliser entre -pi et pi
            if yaw_rad > math.pi:
                yaw_rad -= 2 * math.pi
            elif yaw_rad < -math.pi:
                yaw_rad += 2 * math.pi
            
            pitch_rad = math.radians(pitch_deg)
            roll_rad = math.radians(roll_deg)
            
            # Conversion Euler → Quaternion (ordre ZYX)
            cy = math.cos(yaw_rad * 0.5)
            sy = math.sin(yaw_rad * 0.5)
            cp = math.cos(pitch_rad * 0.5)
            sp = math.sin(pitch_rad * 0.5)
            cr = math.cos(roll_rad * 0.5)
            sr = math.sin(roll_rad * 0.5)
            
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps_link'
            
            msg.orientation.w = cr * cp * cy + sr * sp * sy
            msg.orientation.x = sr * cp * cy - cr * sp * sy
            msg.orientation.y = cr * sp * cy + sr * cp * sy
            msg.orientation.z = cr * cp * sy - sr * sp * cy
            
            # Orientation covariance
            msg.orientation_covariance[0] = 0.001
            msg.orientation_covariance[4] = 0.001
            msg.orientation_covariance[8] = 0.001
            
            self.heading_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Erreur parsing HPR: {e}")
        
        # try:
        #     heading_deg = float(parts[2])
        #     pitch_deg = float(parts[3]) if len(parts) > 3 and parts[3] else 0.0
        #     roll_deg = float(parts[4]) if len(parts) > 4 and parts[4] else 0.0
            
        #     # HPR : Heading, Pitch, Roll en degrés
        #     yaw_rad = math.radians(heading_deg)
        #     pitch_rad = math.radians(pitch_deg)
        #     roll_rad = math.radians(roll_deg)
            
        #     # Conversion Euler → Quaternion (ordre ZYX)
        #     cy = math.cos(yaw_rad * 0.5)
        #     sy = math.sin(yaw_rad * 0.5)
        #     cp = math.cos(pitch_rad * 0.5)
        #     sp = math.sin(pitch_rad * 0.5)
        #     cr = math.cos(roll_rad * 0.5)
        #     sr = math.sin(roll_rad * 0.5)
            
        #     msg = Imu()
        #     msg.header.stamp = self.get_clock().now().to_msg()
        #     msg.header.frame_id = 'gps_link'
            
        #     msg.orientation.w = cr * cp * cy + sr * sp * sy
        #     msg.orientation.x = sr * cp * cy - cr * sp * sy
        #     msg.orientation.y = cr * sp * cy + sr * cp * sy
        #     msg.orientation.z = cr * cp * sy - sr * sp * cy
            
        #     # Orientation covariance
        #     msg.orientation_covariance[0] = 0.001
        #     msg.orientation_covariance[4] = 0.001
        #     msg.orientation_covariance[8] = 0.001
            
        #     self.heading_pub.publish(msg)
            
        # except Exception as e:
        #     self.get_logger().error(f"Erreur parsing HPR: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CustomGpsDriver()
    
    # 1. Vérification explicite avec log d'erreur avant le spin
    if not hasattr(node, 'serial_port') or not node.serial_port.is_open:
        node.get_logger().error(
            "Impossible de lancer le spin : le port série n'est pas initialisé ou est fermé. "
            "Vérifie la connexion ou la règle udev pour /dev/ttyUSB_GPS."
        )
        node.destroy_node()
        rclpy.shutdown()
        return # On quitte proprement immédiatement

    # 2. Encapsulation pour intercepter proprement l'arrêt
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Utilise print() au lieu de node.get_logger().info()
        print("\n[INFO] Arrêt du noeud GPS demandé par l'utilisateur.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()