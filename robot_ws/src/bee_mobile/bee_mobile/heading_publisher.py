#!/usr/bin/env python3
"""
Heading Publisher - Convertit le cap GPS (NED) en cap ROS (ENU)
et le publie en tant que message Imu pour l'EKF.
"""

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped, Quaternion

def yaw_from_quaternion(q):
    """Extrait l'angle de lacet (yaw) en radians à partir d'un quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def quaternion_from_yaw(yaw):
    """Crée un quaternion à partir d'un angle de lacet (yaw) en radians."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class HeadingPublisher(Node):
    def __init__(self):
        super().__init__('heading_publisher')
        
        # Souscrire à /heading (Cap brut du GPS en norme NED)
        self.heading_sub = self.create_subscription(
            QuaternionStamped,
            '/heading',
            self.heading_callback,
            10)
        
        # Publier le heading converti pour les EKF
        self.heading_imu_pub = self.create_publisher(Imu, '/heading_imu', 10)
        
        self.get_logger().info('Heading Publisher (Convertisseur NED -> ENU) démarré')
    
    def heading_callback(self, msg):
        # 1. Extraire l'angle actuel du GPS (Yaw en NED)
        yaw_ned = yaw_from_quaternion(msg.quaternion)
        
        # 2. Appliquer la formule de conversion NED vers ENU
        # Formule : Yaw_ENU = (Pi / 2) - Yaw_NED
        yaw_enu = (math.pi / 2.0) - yaw_ned
        
        # Normaliser l'angle pour qu'il reste entre -Pi et Pi (standard mathématique)
        yaw_enu = (yaw_enu + math.pi) % (2.0 * math.pi) - math.pi
        
        # 3. Créer le nouveau message Imu compréhensible par ROS
        imu_msg = Imu()
        imu_msg.header.stamp = msg.header.stamp
        imu_msg.header.frame_id = 'base_link'
        
        # Convertir le nouvel angle ENU en quaternion
        imu_msg.orientation = quaternion_from_yaw(yaw_enu)
        
        # Matrices de covariance (gardées comme tu les avais définies)
        imu_msg.orientation_covariance = [0.05, 0.0, 0.0, 
                                            0.0, 0.05, 0.0, 
                                            0.0, 0.0, 0.05]
        
        # On dit à l'EKF d'ignorer ces données (-1.0 = donnée invalide/inexistante)
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0
        
        # 4. Publier sur le topic
        self.heading_imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HeadingPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()