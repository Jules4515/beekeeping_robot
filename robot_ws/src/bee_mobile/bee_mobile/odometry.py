#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

class SwerveOdometryNode(Node):
    def __init__(self):
        super().__init__('swerve_odometry_pure_node')

        self.wheel_radius = 0.215
        
        self.wheels = {
            'front_left':  {'x': 0.48,  'y': 0.4150,  'enc_dir': 1.0,  'current_rpm': 0.0, 'current_angle': 0.0},
            'front_right': {'x': 0.48,  'y': -0.4150, 'enc_dir': -1.0, 'current_rpm': 0.0, 'current_angle': 0.0},
            'rear_left':   {'x': -0.48, 'y': 0.4150,  'enc_dir': 1.0,  'current_rpm': 0.0, 'current_angle': 0.0},
            'rear_right':  {'x': -0.48, 'y': -0.4150, 'enc_dir': -1.0, 'current_rpm': 0.0, 'current_angle': 0.0},
        }

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Float64MultiArray, '/mobile/wheel_front_left/encoder_angle',  self.fl_callback, 10)
        self.create_subscription(Float64MultiArray, '/mobile/wheel_front_right/encoder_angle', self.fr_callback, 10)
        self.create_subscription(Float64MultiArray, '/mobile/wheel_rear_left/encoder_angle',   self.rl_callback, 10)
        self.create_subscription(Float64MultiArray, '/mobile/wheel_rear_right/encoder_angle',  self.rr_callback, 10)

        self.odom_timer = self.create_timer(0.02, self.compute_and_publish_odometry)

    def update_wheel_state(self, wheel_name, msg):
        if len(msg.data) >= 2:
            self.wheels[wheel_name]['current_rpm'] = float(msg.data[0]) * self.wheels[wheel_name]['enc_dir']
            self.wheels[wheel_name]['current_angle'] = float(msg.data[1])

    def fl_callback(self, msg): self.update_wheel_state('front_left', msg)
    def fr_callback(self, msg): self.update_wheel_state('front_right', msg)
    def rl_callback(self, msg): self.update_wheel_state('rear_left', msg)
    def rr_callback(self, msg): self.update_wheel_state('rear_right', msg)

    def compute_and_publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        # ==========================================================
        # PASS 1: Décomposition Vectorielle Strictement Physique
        # ==========================================================
        vx_sum, vy_sum = 0.0, 0.0

        for config in self.wheels.values():
            # Vitesse et Angle Physiques Bruts
            speed_ms = (config['current_rpm'] * 2.0 * math.pi * self.wheel_radius) / 60.0
            angle_rad = math.radians(config['current_angle'])
            
            # Projection Vectorielle
            config['vx_w'] = speed_ms * math.cos(angle_rad)
            config['vy_w'] = speed_ms * math.sin(angle_rad)
            vx_sum += config['vx_w']
            vy_sum += config['vy_w']

        # Vitesses linéaires réelles du châssis
        real_vx_robot = vx_sum / 4.0
        real_vy_robot = vy_sum / 4.0

        # ==========================================================
        # PASS 2: Calcul de la Vitesse Angulaire (wz)
        # ==========================================================
        wz_num, wz_den = 0.0, 0.0

        for config in self.wheels.values():
            wz_num += (config['x'] * (config['vy_w'] - real_vy_robot) - config['y'] * (config['vx_w'] - real_vx_robot))
            wz_den += config['x']**2 + config['y']**2

        real_wz_robot = wz_num / wz_den if wz_den > 0 else 0.0

        # ==========================================================
        # PASS 3: Intégration Spatiale
        # ==========================================================
        delta_theta = real_wz_robot * dt
        theta_mid = self.odom_theta + delta_theta / 2.0 
        
        self.odom_x += (real_vx_robot * math.cos(theta_mid) - real_vy_robot * math.sin(theta_mid)) * dt
        self.odom_y += (real_vx_robot * math.sin(theta_mid) + real_vy_robot * math.cos(theta_mid)) * dt
        self.odom_theta += delta_theta

        # ==========================================================
        # PUBLICATION: TF Tree & Odom (100% Physique)
        # ==========================================================
        tf_time = current_time + rclpy.duration.Duration(seconds=0.10) 
        cy = math.cos(self.odom_theta * 0.5)
        sy = math.sin(self.odom_theta * 0.5)

        # TF Broadcast
        t = TransformStamped()
        t.header.stamp = tf_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        self.tf_broadcaster.sendTransform(t)

        # Odometry Message
        odom = Odometry()
        odom.header.stamp = tf_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # POSE (Vérité spatiale)
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        
        # TWIST (Vérité cinématique)
        odom.twist.twist.linear.x = real_vx_robot
        odom.twist.twist.linear.y = real_vy_robot
        odom.twist.twist.angular.z = real_wz_robot
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()