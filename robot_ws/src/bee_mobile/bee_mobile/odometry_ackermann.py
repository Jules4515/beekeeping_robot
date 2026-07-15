#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdometryAckermann(Node):
    def __init__(self):
        super().__init__('odometry_ackermann')

        self.wheel_radius = 0.215
        
        self.wheels = {
            'front_left':  {'x': 0.48,  'y': 0.4150,  'enc_dir': 1.0,  'current_rpm': 0.0, 'current_angle': 0.0},
            'front_right': {'x': 0.48,  'y': -0.4150, 'enc_dir': -1.0, 'current_rpm': 0.0, 'current_angle': 0.0},
        }

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_time = self.get_clock().now()

        # Paramètre pour activer/désactiver la TF
        self.declare_parameter('publish_odom_tf', False)
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # SUPPRESSION DU TF BROADCASTER (C'est l'EKF qui gère ça maintenant)

        self.create_subscription(Float64MultiArray, '/mobile/wheel_front_left/encoder_angle',  self.fl_callback, 10)
        self.create_subscription(Float64MultiArray, '/mobile/wheel_front_right/encoder_angle', self.fr_callback, 10)

        self.odom_timer = self.create_timer(0.02, self.compute_and_publish_odometry)

    def update_wheel_state(self, wheel_name, msg):
        if len(msg.data) >= 2:
            self.wheels[wheel_name]['current_rpm'] = float(msg.data[0]) * self.wheels[wheel_name]['enc_dir']
            self.wheels[wheel_name]['current_angle'] = float(msg.data[1])

    def fl_callback(self, msg): self.update_wheel_state('front_left', msg)
    def fr_callback(self, msg): self.update_wheel_state('front_right', msg)

    def compute_and_publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        # ==========================================================
        # MODÈLE BICYCLETTE (Ackermann FWD projeté)
        # ==========================================================
        L_empattement = 0.96  # Distance entre l'essieu arrière (fixe) et avant (directeur)

        # 1. Extraction des vitesses et angles individuels des roues avant
        v_fl = (self.wheels['front_left']['current_rpm'] * 2.0 * math.pi * self.wheel_radius) / 60.0
        angle_fl = math.radians(self.wheels['front_left']['current_angle'])
        
        v_fr = (self.wheels['front_right']['current_rpm'] * 2.0 * math.pi * self.wheel_radius) / 60.0
        angle_fr = math.radians(self.wheels['front_right']['current_angle'])

        # 2. Projection des vecteurs cinématiques sur l'essieu avant
        # On calcule la vitesse moyenne de l'essieu avant sur ses axes X et Y
        v_xf = (v_fl * math.cos(angle_fl) + v_fr * math.cos(angle_fr)) / 2.0
        v_yf = (v_fl * math.sin(angle_fl) + v_fr * math.sin(angle_fr)) / 2.0

        # 3. Calcul de la cinématique globale du châssis
        # La vitesse longitudinale du robot est dictée par l'avancement de l'essieu avant
        real_vx_robot = v_xf
        
        # La rotation du châssis est créée par la vitesse latérale de l'essieu avant autour de l'essieu arrière
        real_wz_robot = v_yf / L_empattement
        
        # Le repère base_footprint est au centre géométrique (à L/2 de l'essieu arrière)
        # Il subit donc une dérive latérale lors d'un virage
        real_vy_robot = real_wz_robot * (L_empattement / 2.0)

        # ==========================================================
        # Intégration Spatiale
        # ==========================================================
        delta_theta = real_wz_robot * dt
        theta_mid = self.odom_theta + delta_theta / 2.0 
        
        self.odom_x += (real_vx_robot * math.cos(theta_mid) - real_vy_robot * math.sin(theta_mid)) * dt
        self.odom_y += (real_vx_robot * math.sin(theta_mid) + real_vy_robot * math.cos(theta_mid)) * dt
        self.odom_theta += delta_theta

        # ==========================================================
        # PUBLICATION: Odom (avec incertitudes pour EKF)
        # ==========================================================
        cy = math.cos(self.odom_theta * 0.5)
        sy = math.sin(self.odom_theta * 0.5)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint' 
        
        # POSE
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        
        # TWIST
        odom.twist.twist.linear.x = real_vx_robot
        odom.twist.twist.linear.y = real_vy_robot
        odom.twist.twist.angular.z = real_wz_robot

        covariance_matrix = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05
        ]
        odom.pose.covariance = covariance_matrix
        odom.twist.covariance = covariance_matrix
        
        self.odom_pub.publish(odom)

        # ==========================================================
        # PUBLICATION TF (Optionnelle)
        # ==========================================================
        if self.publish_odom_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            
            t.transform.translation.x = self.odom_x
            t.transform.translation.y = self.odom_y
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = sy
            t.transform.rotation.w = cy
            
            self.tf_broadcaster.sendTransform(t)
            
def main(args=None):
    rclpy.init(args=args)
    node = OdometryAckermann()
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