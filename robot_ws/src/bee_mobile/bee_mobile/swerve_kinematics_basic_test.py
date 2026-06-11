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
            'front_right': {'x': 0.48,  'y': -0.4150, 'dir': 1.0, 'enc_dir': -1.0, 'current_angle': 0.0, 'current_rpm': 0.0, 'last_angle': 0.0, 'last_speed': 0.0},
            'rear_left':   {'x': -0.48, 'y': 0.4150,  'dir': 1.0, 'enc_dir': 1.0,  'current_angle': 0.0, 'current_rpm': 0.0, 'last_angle': 0.0, 'last_speed': 0.0},
            'rear_right':  {'x': -0.48, 'y': -0.4150, 'dir': 1.0, 'enc_dir': -1.0, 'current_angle': 0.0, 'current_rpm': 0.0, 'last_angle': 0.0, 'last_speed': 0.0},
        }

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()  # Dedicated timer for Slew Rate Calculation

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

    def cmd_vel_callback(self, msg):
        # Calculate time elapsed since last command to compute physically achievable angle steps
        current_time = self.get_clock().now()
        dt = (current_time - self.last_cmd_time).nanoseconds / 1e9
        self.last_cmd_time = current_time
        
        # Prevent massive dt spikes on first run or after long pauses
        if dt <= 0.0 or dt > 0.5:
            dt = 0.1

        # Physical machine limits
        MAX_STEER_RAD_S = 0.785  # Max servo rotation speed (~45 deg/s) - Tune based on hardware
        MAX_RPM_LIMIT = 30.0  # Limite de sécurité physique
        MAX_SPEED_MS = (MAX_RPM_LIMIT * 2.0 * math.pi * self.wheel_radius) / 60.0
        HARD_LIMIT_RAD = math.radians(80.0) # Limite physique de +-80 deg pour l'orientation des roues
        
        # --- PARAMÈTRES DU PROFIL DE TRACTION ---
        TOLERANCE_VERTE_DEG = 10.0  # Fin de la zone 100%
        TOLERANCE_ROUGE_DEG = 40.0  # Début de la zone 0%
        
        # Conversion en radians pour le calcul interne
        TOLERANCE_VERTE_RAD = math.radians(TOLERANCE_VERTE_DEG)
        TOLERANCE_ROUGE_RAD = math.radians(TOLERANCE_ROUGE_DEG)

        for name, config in self.wheels.items():
            vx_w = msg.linear.x - config['y'] * msg.angular.z
            vy_w = msg.linear.y + config['x'] * msg.angular.z
            
            raw_speed = math.hypot(vx_w, vy_w)
            raw_angle = math.atan2(vy_w, vx_w) if raw_speed > 0.001 else config['last_angle']

            # 1. Absolute Phase Inversion (Clamp to +/- 90 deg)
            if raw_angle > (math.pi / 2.0):
                ideal_angle = raw_angle - math.pi
                ideal_speed = -raw_speed
            elif raw_angle < -(math.pi / 2.0):
                ideal_angle = raw_angle + math.pi
                ideal_speed = -raw_speed
            else:
                ideal_angle = raw_angle
                ideal_speed = raw_speed

            # 2. Slew Rate Limiter: Ramp the angle target instead of instant snapping
            angle_diff = ideal_angle - config['last_angle']
            # Normalisation stricte de la différence pour le chemin le plus court
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            max_step = MAX_STEER_RAD_S * dt

            if angle_diff > max_step:
                cmd_angle = config['last_angle'] + max_step
            elif angle_diff < -max_step:
                cmd_angle = config['last_angle'] - max_step
            else:
                cmd_angle = ideal_angle

            # 3. Verrou de Traction Proportionnel (Profil Quadratique Convexe)
            current_phys_angle = math.radians(config['current_angle'])
            error_to_ideal = ideal_angle - current_phys_angle
            
            # Normalisation au chemin le plus court pour éviter le wrap-around de Pi
            error_to_ideal = abs(math.atan2(math.sin(error_to_ideal), math.cos(error_to_ideal)))

            if error_to_ideal <= TOLERANCE_VERTE_RAD:
                speed_multiplier = 1.0
            elif error_to_ideal >= TOLERANCE_ROUGE_RAD:
                speed_multiplier = 0.0
            else:
                # Calcul du ratio linéaire de 0.0 à 1.0 dans la zone de dégradation
                ratio = (error_to_ideal - TOLERANCE_VERTE_RAD) / (TOLERANCE_ROUGE_RAD - TOLERANCE_VERTE_RAD)
                
                # Application du profil Quadratique Convexe : (1 - ratio)^2
                speed_multiplier = (1.0 - ratio) ** 2

            # Application finale à la consigne de vitesse
            cmd_speed = ideal_speed * speed_multiplier

            # Application de la limite physique d'angle sur l'orientation des roues (Clamping)
            cmd_angle = max(min(cmd_angle, HARD_LIMIT_RAD), -HARD_LIMIT_RAD)
            # Application de la limite physique sur la vitesse des roues (Clamping)
            cmd_speed = max(min(cmd_speed, MAX_SPEED_MS), -MAX_SPEED_MS)

            # Update state variables
            config['last_angle'] = cmd_angle
            config['last_speed'] = cmd_speed

            # Conversion and Publication
            rpm_final = (cmd_speed * 60.0) / (2.0 * math.pi * self.wheel_radius) * config['dir']            
            self.wheel_pubs[name].publish(Float64MultiArray(data=[float(rpm_final), float(math.degrees(cmd_angle))]))

    def odometry_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0:
            return

        # Pass 1 — linear velocities (accumulate all 4 wheels first)
        vx_sum, vy_sum = 0.0, 0.0
        for config in self.wheels.values():
            speed_ms = (config['current_rpm'] * 2.0 * math.pi * self.wheel_radius) / 60.0
            angle_rad = math.radians(config['current_angle'])
            config['vx_w'] = speed_ms * math.cos(angle_rad)
            config['vy_w'] = speed_ms * math.sin(angle_rad)
            vx_sum += config['vx_w']
            vy_sum += config['vy_w']

        vx_robot = vx_sum / 4.0
        vy_robot = vy_sum / 4.0

        # Pass 2 — angular velocity using the FINAL average (not partial)
        wz_num, wz_den = 0.0, 0.0
        for config in self.wheels.values():
            wz_num += (config['x'] * (config['vy_w'] - vy_robot)
                    - config['y'] * (config['vx_w'] - vx_robot))
            wz_den += config['x']**2 + config['y']**2

        wz_robot = wz_num / wz_den if wz_den > 0 else 0.0

        delta_theta = wz_robot * dt
        theta_mid = self.odom_theta + delta_theta / 2.0
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