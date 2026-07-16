#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Int8
import math

class KinematicsAckermann(Node):
    def __init__(self):
        super().__init__('kinematics_ackermann')
        
        # --- 1. INITIALISATION DES VARIABLES & PARAMÈTRES ---
        self._init_parameters()
        self._init_kinematic_constants()
        self._init_wheel_hardware()
        self._init_ros_interfaces()

    def _init_parameters(self):
        """Loads and converts ROS 2 parameters to class variables."""
        self.declare_parameter('max_steer_deg_s', 80.0)
        self.declare_parameter('hard_limit_deg', 50.0)
        self.declare_parameter('v_max_physical', 0.80)
        self.declare_parameter('v_min_moteur', 0.50)
        self.declare_parameter('v_min_nav2', 0.10)
        self.declare_parameter('deadband_vx', 0.05)
        self.declare_parameter('deadband_wz', 0.05)
        
        # Paramètres du Slew Rate Linéaire
        self.declare_parameter('enable_speed_slew_rate', True)
        self.declare_parameter('accel_max', 0.50)
        self.declare_parameter('decel_max', 2.0)

        self.max_steer_rad_s = math.radians(self.get_parameter('max_steer_deg_s').value)
        self.hard_limit_rad = math.radians(self.get_parameter('hard_limit_deg').value)
        self.v_max_physical = self.get_parameter('v_max_physical').value
        self.v_min_moteur = self.get_parameter('v_min_moteur').value
        self.v_min_nav2 = self.get_parameter('v_min_nav2').value
        self.deadband_vx = self.get_parameter('deadband_vx').value
        self.deadband_wz = self.get_parameter('deadband_wz').value

        self.enable_speed_slew_rate = self.get_parameter('enable_speed_slew_rate').value
        self.accel_max = self.get_parameter('accel_max').value
        self.decel_max = self.get_parameter('decel_max').value

    def _init_kinematic_constants(self):
        """Pre-computes mapping coefficients to save CPU cycles in the main loop."""
        self.wheel_radius = 0.215
        
        v_max = self.v_max_physical
        v_min_mot = self.v_min_moteur
        v_min_nav = self.v_min_nav2
        
        self.map_m = (v_max - v_min_mot) / (v_max - v_min_nav)
        self.map_A = (2.0 * (v_min_mot - self.map_m * v_min_nav)) / math.sqrt(v_min_nav)
        self.map_B = 2.0 * self.map_m - (v_min_mot / v_min_nav)

    def _init_wheel_hardware(self):
        """Defines the physical geometry and state matrix of the Swerve drive."""
        self.wheels = {
            'front_left':  {'x': 0.48,  'y': 0.4150,  'dir': 1.0, 'enc_dir': 1.0,  'current_angle': 0.0, 'current_rpm': 0.0, 'last_angle': 0.0, 'last_target_angle': 0.0, 'last_logical_speed': 0.0},
            'front_right': {'x': 0.48,  'y': -0.4150, 'dir': 1.0, 'enc_dir': -1.0, 'current_angle': 0.0, 'current_rpm': 0.0, 'last_angle': 0.0, 'last_target_angle': 0.0, 'last_logical_speed': 0.0},
        }

    def _init_ros_interfaces(self):
        """Sets up ROS 2 publishers, subscribers, and timers."""
        self.last_cmd_time = self.get_clock().now()

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_out', self.cmd_vel_callback, 10)
        
        self.wheel_pubs = {}

        for name in self.wheels.keys():
            self.wheel_pubs[name] = self.create_publisher(Float64MultiArray, f'mobile/wheel_{name}/motor_speed', 10)
            self.create_subscription(Float64MultiArray, f'mobile/wheel_{name}/encoder_angle', 
                                     lambda msg, n=name: self.encoder_callback(msg, n), 10)
            
    # --- 2. CALLBACKS ---
    def encoder_callback(self, msg, wheel_name):
        if len(msg.data) >= 2:
            self.wheels[wheel_name]['current_rpm'] = float(msg.data[0]) * self.wheels[wheel_name]['enc_dir']
            self.wheels[wheel_name]['current_angle'] = float(msg.data[1])

    def cmd_vel_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_cmd_time).nanoseconds / 1e9
        self.last_cmd_time = current_time

        if dt <= 0.0 or dt > 0.5:
            dt = 0.1

        vx = msg.linear.x
        wz = msg.angular.z

        # Application de la zone morte (Deadband)
        if abs(vx) < self.deadband_vx: vx = 0.0
        if abs(wz) < self.deadband_wz: wz = 0.0

        # On calcule les cibles et on les publie immédiatement
        self._calculate_wheel_targets(vx, wz, dt)
        self._publish_hardware_commands(dt)

    # --- 3. MÉTHODES CINÉMATIQUES INTERNES ---
    def _calculate_wheel_targets(self, vx, wz, dt):
        """Computes true Ackermann angles and speeds relative to the rear axle."""
        L_empattement = 0.96 
        
        # Rayon de braquage minimum physique (ex: 0.96 / tan(50°) = 0.805 m)
        R_min = L_empattement / math.tan(self.hard_limit_rad)

        for name, config in self.wheels.items():
            if vx == 0.0 and wz == 0.0:
                ideal_speed = 0.0
                ideal_angle = config['last_target_angle'] 
                
            elif vx == 0.0 and wz != 0.0:
                # --- DRY STEERING (Braquage à l'arrêt) ---
                # A l'arrêt, le joystick droit agit comme un volant physique direct.
                # Wz est normalisé (par rapport à une valeur arbitraire de joystick ex: 0.50)
                # pour offrir une réactivité parfaite sans devoir pousser le stick à fond.
                steer_ratio = max(min(wz / 0.50, 1.0), -1.0)
                ideal_angle = steer_ratio * self.hard_limit_rad
                ideal_speed = 0.0
                
            else:
                # --- ACKERMANN DYNAMIQUE ---
                # Protection : On bride Wz pour ne jamais demander un rayon < R_min
                max_wz_allowed = abs(vx) / R_min
                clamped_wz = max(min(wz, max_wz_allowed), -max_wz_allowed)

                # Calcul strict de l'angle d'Ackermann pour la roue intérieure/extérieure
                denom = vx - config['y'] * clamped_wz
                
                # Sécurité mathématique (division par zéro)
                if abs(denom) < 0.001:
                    denom = math.copysign(0.001, denom)
                    
                # L'utilisation de math.atan gère nativement la marche arrière
                # car un vx négatif inversera logiquement l'angle de braquage.
                ideal_angle = math.atan( (L_empattement * clamped_wz) / denom )
                
                # La vitesse est la norme du vecteur tangentiel
                vy_w = L_empattement * clamped_wz
                vx_w = vx - config['y'] * clamped_wz
                ideal_speed = math.hypot(vx_w, vy_w)
                
                # On applique le sens de la marche (Marche Avant / Marche Arrière)
                ideal_speed = math.copysign(ideal_speed, vx)

            config['last_target_angle'] = ideal_angle

            # Steer Slew Rate Limiter (Limitation matérielle de la vitesse de braquage)
            angle_diff = ideal_angle - config['last_angle']
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            max_step_angle = self.max_steer_rad_s * dt

            if angle_diff > max_step_angle:
                cmd_angle = config['last_angle'] + max_step_angle
            elif angle_diff < -max_step_angle:
                cmd_angle = config['last_angle'] - max_step_angle
            else:
                cmd_angle = ideal_angle

            # Saturation de sécurité (Hard Limit)
            cmd_angle = max(min(cmd_angle, self.hard_limit_rad), -self.hard_limit_rad)
            
            config['temp_cmd_angle'] = cmd_angle
            config['temp_ideal_speed'] = ideal_speed

    def _publish_hardware_commands(self, dt):
        """Applies linear slew rate, non-linear speed mapping, and publishes commands."""
        precalculated_commands = {}

        for name, config in self.wheels.items():
            cmd_angle = config['temp_cmd_angle']
            
            # Application directe de la vitesse (sans bridage d'alignement)
            ideal_speed = config['temp_ideal_speed']

            logical_target_speed = max(min(ideal_speed, self.v_max_physical), -self.v_max_physical)

            # ============================================================
            # Linar Speed Slew Rate Limiter
            # ============================================================
            last_logical_spd = config['last_logical_speed']
            speed_diff = logical_target_speed - last_logical_spd
            
            is_accelerating = abs(logical_target_speed) > abs(last_logical_spd) or (logical_target_speed * last_logical_spd < 0)
            max_step_lin = (self.accel_max if is_accelerating else self.decel_max) * dt
                
            if abs(speed_diff) > max_step_lin:
                logical_smoothed_speed = last_logical_spd + math.copysign(max_step_lin, speed_diff)
            else:
                logical_smoothed_speed = logical_target_speed

            config['last_logical_speed'] = logical_smoothed_speed
            
            # ============================================================
            # Hybrid Speed Mapping
            # ============================================================
            abs_spd = abs(logical_smoothed_speed)
            if abs_spd > 0.005:
                sign = math.copysign(1.0, logical_smoothed_speed)
                
                if abs_spd <= self.v_min_nav2:
                    cmd_speed_hardware = sign * (self.map_A * math.sqrt(abs_spd) + self.map_B * abs_spd)
                else:
                    cmd_speed_hardware = sign * (self.map_m * (abs_spd - self.v_min_nav2) + self.v_min_moteur)
            else:
                cmd_speed_hardware = 0.0
                
            cmd_speed_hardware = max(min(cmd_speed_hardware, self.v_max_physical), -self.v_max_physical)
            
            config['last_angle'] = cmd_angle

            # Publish
            rpm_final = (cmd_speed_hardware * 60.0) / (2.0 * math.pi * self.wheel_radius) * config['dir']   
            
            # Sauvegarde dans le dictionnaire local
            precalculated_commands[name] = {
                'rpm': rpm_final,
                'angle_deg': math.degrees(cmd_angle),
            }

        for name, cmds in precalculated_commands.items():
            self.wheel_pubs[name].publish(Float64MultiArray(data=[float(cmds['rpm']), float(cmds['angle_deg'])]))
            
def main(args=None):
    rclpy.init(args=args)
    node = KinematicsAckermann()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()