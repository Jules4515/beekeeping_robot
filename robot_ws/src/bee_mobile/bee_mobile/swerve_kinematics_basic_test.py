#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
import math

class SwerveKinematicsMVP(Node):
    def __init__(self):
        super().__init__('swerve_kinematics_node')

        self.wheel_radius = 0.215

        self.wheels = {
            'front_left':  {'x': 0.48,  'y': 0.4150,  'dir': 1.0, 'enc_dir': 1.0,  'current_angle': 0.0, 'current_rpm': 0.0, 'last_angle': 0.0, 'last_speed': 0.0},
            'front_right': {'x': 0.48,  'y': -0.4150, 'dir': -1.0, 'enc_dir': -1.0, 'current_angle': 0.0, 'current_rpm': 0.0, 'last_angle': 0.0, 'last_speed': 0.0},
            'rear_left':   {'x': -0.48, 'y': 0.4150,  'dir': 1.0, 'enc_dir': 1.0,  'current_angle': 0.0, 'current_rpm': 0.0, 'last_angle': 0.0, 'last_speed': 0.0},
            'rear_right':  {'x': -0.48, 'y': -0.4150, 'dir': -1.0, 'enc_dir': -1.0, 'current_angle': 0.0, 'current_rpm': 0.0, 'last_angle': 0.0, 'last_speed': 0.0},
        }

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_time = self.get_clock().now()

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_out', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.wheel_pubs = {}
        for name in self.wheels.keys():
            self.wheel_pubs[name] = self.create_publisher(Float64MultiArray, f'mobile/wheel_{name}/motor_speed', 10)
            self.create_subscription(Float64MultiArray, f'mobile/wheel_{name}/encoder_angle', lambda msg, n=name: self.encoder_callback(msg, n), 10)

        self.odom_timer = self.create_timer(0.02, self.odometry_callback)

    def encoder_callback(self, msg, wheel_name):
        if len(msg.data) >= 2:
            self.wheels[wheel_name]['current_rpm'] = float(msg.data[0]) * self.wheels[wheel_name]['enc_dir']
            self.wheels[wheel_name]['current_angle'] = float(msg.data[1])

    def _normalize_angle(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def cmd_vel_callback(self, msg):
        for name, config in self.wheels.items():
            # 1. Cinématique Inverse Pure
            vx_w = msg.linear.x - config['y'] * msg.angular.z
            vy_w = msg.linear.y + config['x'] * msg.angular.z
            
            raw_speed = math.hypot(vx_w, vy_w)
            raw_angle = math.atan2(vy_w, vx_w) if raw_speed > 0.001 else config['last_angle']

            # 2. Optimisation basique du plus court chemin (Évite l'arrachement des câbles)
            diff = self._normalize_angle(raw_angle - config['last_angle'])
            if abs(diff) > (math.pi / 2.0):
                target_angle = self._normalize_angle(raw_angle - math.pi)
                target_speed = -raw_speed
            else:
                target_angle = raw_angle
                target_speed = raw_speed

            # 3. Mise à jour de l'état
            config['last_angle'] = target_angle
            config['last_speed'] = target_speed

            # 4. Conversion et Publication immédiate (Aucun délai)
            rpm = (target_speed * 60.0) / (2.0 * math.pi * self.wheel_radius) * config['dir']
            self.wheel_pubs[name].publish(Float64MultiArray(data=[float(rpm), float(math.degrees(target_angle))]))

    def odometry_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0: return

        vx_sum, vy_sum = 0.0, 0.0
        wz_num, wz_den = 0.0, 0.0

        for config in self.wheels.values():
            speed_ms = (config['current_rpm'] * 2.0 * math.pi * self.wheel_radius) / 60.0
            angle_rad = math.radians(config['current_angle'])

            vx_w = speed_ms * math.cos(angle_rad)
            vy_w = speed_ms * math.sin(angle_rad)
            
            vx_sum += vx_w
            vy_sum += vy_w

            wz_num += config['x'] * (vy_w - (vy_sum/4.0)) - config['y'] * (vx_w - (vx_sum/4.0))
            wz_den += config['x']**2 + config['y']**2

        vx_robot = vx_sum / 4.0
        vy_robot = vy_sum / 4.0
        wz_robot = wz_num / wz_den if wz_den > 0 else 0.0

        delta_theta = wz_robot * dt
        theta_mid = self.odom_theta + (delta_theta / 2.0)
        self.odom_x += (vx_robot * math.cos(theta_mid) - vy_robot * math.sin(theta_mid)) * dt
        self.odom_y += (vx_robot * math.sin(theta_mid) + vy_robot * math.cos(theta_mid)) * dt
        self.odom_theta += delta_theta

        self.publish_odometry(current_time, vx_robot, vy_robot, wz_robot)

    def publish_odometry(self, current_time, vx, vy, wz):
        tf_time = current_time + rclpy.duration.Duration(seconds=0.10)
        cy = math.cos(self.odom_theta * 0.5)
        sy = math.sin(self.odom_theta * 0.5)

        t = TransformStamped()
        t.header.stamp = tf_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = self.odom_x, self.odom_y
        t.transform.rotation.z, t.transform.rotation.w = sy, cy
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = tf_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.odom_x, self.odom_y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = sy, cy
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = vx, vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SwerveKinematicsMVP())
    rclpy.shutdown()

if __name__ == '__main__':
    main()