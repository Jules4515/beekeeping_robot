#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Int8, Float64
import math

class SwerveKinematics(Node):
    def __init__(self):
        super().__init__('swerve_kinematics')

        self.wheel_radius = 0.215

        # si sim mettre des - pour dir pour les right wheels
        self.wheels = {
            'front_left':  {'x': 0.48,  'y': 0.4150,  'dir': 1.0, 'enc_dir': 1.0,  'current_angle': 0.0, 'current_rpm': 0.0, 'last_angle': 0.0, 'last_speed': 0.0, 'last_logical_speed': 0.0, 'stop_counter': 0, 'confirmed_stopped': True},
            'front_right': {'x': 0.48,  'y': -0.4150, 'dir': 1.0, 'enc_dir': -1.0, 'current_angle': 0.0, 'current_rpm': 0.0, 'last_angle': 0.0, 'last_speed': 0.0, 'last_logical_speed': 0.0, 'stop_counter': 0, 'confirmed_stopped': True},
            'rear_left':   {'x': -0.48, 'y': 0.4150,  'dir': 1.0, 'enc_dir': 1.0,  'current_angle': 0.0, 'current_rpm': 0.0, 'last_angle': 0.0, 'last_speed': 0.0, 'last_logical_speed': 0.0, 'stop_counter': 0, 'confirmed_stopped': True},
            'rear_right':  {'x': -0.48, 'y': -0.4150, 'dir': 1.0, 'enc_dir': -1.0, 'current_angle': 0.0, 'current_rpm': 0.0, 'last_angle': 0.0, 'last_speed': 0.0, 'last_logical_speed': 0.0, 'stop_counter': 0, 'confirmed_stopped': True},
        }

        self.last_cmd_time = self.get_clock().now()  # Dedicated timer for Slew Rate Calculation

        # --- État du Joystick ---
        self.active_joy_mode = -1
        self.mode_sub = self.create_subscription(Int8, '/joystick_control_mode', self.mode_callback, 10)

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_out', self.cmd_vel_callback, 10)
        
        self.wheel_pubs = {}
        self.logical_speed_pubs = {}

        for name in self.wheels.keys():
            self.wheel_pubs[name] = self.create_publisher(Float64MultiArray, f'mobile/wheel_{name}/motor_speed', 10)
            self.logical_speed_pubs[name] = self.create_publisher(Float64, f'mobile/wheel_{name}/logical_speed', 10)
            self.create_subscription(Float64MultiArray, f'mobile/wheel_{name}/encoder_angle', lambda msg, n=name: self.encoder_callback(msg, n), 10)

    def mode_callback(self, msg):
        self.active_joy_mode = msg.data

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
        MAX_STEER_RAD_S = math.radians(90.0)  # Max servo rotation SPEED (~45 deg/s) - Tune based on hardware
        MAX_RPM_LIMIT = 30.0  # Limite de sécurité physique
        MAX_SPEED_MS = (MAX_RPM_LIMIT * 2.0 * math.pi * self.wheel_radius) / 60.0
        HARD_LIMIT_RAD = math.radians(80.0) # Limite physique de +-80 deg pour l'orientation des roues
        
        # --- PARAMÈTRES DU PROFIL DE TRACTION ---
        TOLERANCE_VERTE_DEG = 10.0  # Fin de la zone 100%
        TOLERANCE_ROUGE_DEG = 40.0  # Début de la zone 0%
        
        # Conversion en radians pour le calcul interne
        TOLERANCE_VERTE_RAD = math.radians(TOLERANCE_VERTE_DEG)
        TOLERANCE_ROUGE_RAD = math.radians(TOLERANCE_ROUGE_DEG)

        # ============================================================
        # NOUVEAU : PRÉ-CALCUL DE L'ERREUR GLOBALE (ANTI-DRAGGING)
        # ============================================================
        max_angle_error_rad = 0.0
        
        for name, config in self.wheels.items():
            vx_w = msg.linear.x - config['y'] * msg.angular.z
            vy_w = msg.linear.y + config['x'] * msg.angular.z
            raw_speed = math.hypot(vx_w, vy_w)
            
            if raw_speed > 0.001:
                raw_angle = math.atan2(vy_w, vx_w)
            else:
                if self.active_joy_mode == 3:      # Zero Turn
                    raw_angle = math.atan2(config['x'], -config['y'])
                elif self.active_joy_mode in [1, 2]: # Straight ou Crab
                    raw_angle = 0.0
                else:                              # Holonome (0) ou inactif (-1)
                    raw_angle = config['last_angle']

            if raw_angle > (math.pi / 2.0):
                ideal_angle = raw_angle - math.pi
            elif raw_angle < -(math.pi / 2.0):
                ideal_angle = raw_angle + math.pi
            else:
                ideal_angle = raw_angle

            current_phys_angle = math.radians(config['current_angle'])
            error_to_ideal = ideal_angle - current_phys_angle
            error_to_ideal = abs(math.atan2(math.sin(error_to_ideal), math.cos(error_to_ideal)))

            if error_to_ideal > max_angle_error_rad:
                max_angle_error_rad = error_to_ideal

        # Verrou de Traction GLOBAL (Bloc déplacé ici)
        if max_angle_error_rad <= TOLERANCE_VERTE_RAD:
            global_speed_multiplier = 1.0
        elif max_angle_error_rad >= TOLERANCE_ROUGE_RAD:
            global_speed_multiplier = 0.0
        else:
            # Calcul du ratio linéaire de 0.0 à 1.0 dans la zone de dégradation
            ratio = (max_angle_error_rad - TOLERANCE_VERTE_RAD) / (TOLERANCE_ROUGE_RAD - TOLERANCE_VERTE_RAD)
            # Application du profil Quadratique Convexe : (1 - ratio)^2
            global_speed_multiplier = (1.0 - ratio) ** 2
        # ============================================================

        for name, config in self.wheels.items():
            vx_w = msg.linear.x - config['y'] * msg.angular.z
            vy_w = msg.linear.y + config['x'] * msg.angular.z
            
            raw_speed = math.hypot(vx_w, vy_w)
            
            # --- Logique de Pré-orientation simplifiée ---
            if raw_speed > 0.001:
                raw_angle = math.atan2(vy_w, vx_w)
            else:
                if self.active_joy_mode == 3:      # Zero Turn
                    raw_angle = math.atan2(config['x'], -config['y'])
                elif self.active_joy_mode in [1, 2]: # Straight ou Crab
                    raw_angle = 0.0
                else:                              # Holonome (0) ou inactif (-1)
                    raw_angle = config['last_angle']

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
            max_step_angle = MAX_STEER_RAD_S * dt

            if angle_diff > max_step_angle:
                cmd_angle = config['last_angle'] + max_step_angle
            elif angle_diff < -max_step_angle:
                cmd_angle = config['last_angle'] - max_step_angle
            else:
                cmd_angle = ideal_angle

            # Application de la limite physique d'angle sur l'orientation des roues (Clamping)
            cmd_angle = max(min(cmd_angle, HARD_LIMIT_RAD), -HARD_LIMIT_RAD)

            # 3. Application du Verrou de Traction GLOBAL (Calculé précédemment)
            speed_multiplier = global_speed_multiplier

            # Application finale à la consigne de vitesse LOGIQUE
            logical_target_speed = ideal_speed * speed_multiplier
            
            V_MAX_PHYSICAL = 0.70  # Ta vitesse maximale
            
            logical_target_speed = max(min(logical_target_speed, V_MAX_PHYSICAL), -V_MAX_PHYSICAL)

            # ============================================================
            # SLEW RATE LIMITER (Accélération Linéaire Constante)
            # ============================================================
            A_MAX = 2.0  # m/s² - Limite d'accélération (À régler)
            D_MAX = 2.0  # m/s² - Limite de décélération (Freinage)
            
            last_logical_spd = config['last_logical_speed']
            speed_diff = logical_target_speed - last_logical_spd
            
            # Détermination de l'état dynamique (Accélération vs Freinage)
            # On accélère si la vitesse absolue augmente OU si on inverse le sens de rotation
            is_accelerating = abs(logical_target_speed) > abs(last_logical_spd) or (logical_target_speed * last_logical_spd < 0)
            
            # Choix du taux limite en fonction de l'état
            max_step_lin = (A_MAX if is_accelerating else D_MAX) * dt
                
            # Application de la limite cinématique
            if abs(speed_diff) > max_step_lin:
                logical_smoothed_speed = last_logical_spd + math.copysign(max_step_lin, speed_diff)
            else:
                logical_smoothed_speed = logical_target_speed
                logical_smoothed_speed
            # ============================================================
            # MAPPING CONTINU C1 (HYBRIDE RACINE / LINÉAIRE)
            # ============================================================
            V_MAX = 0.70
            V_MIN_MOTEUR = 0.30
            V_MIN_NAV2 = 0.10
            
            abs_smoothed = abs(logical_smoothed_speed)
            
            if abs_smoothed > 0.005:
                sign = math.copysign(1.0, logical_smoothed_speed)
                
                # Pré-calcul des coefficients (À déplacer dans le __init__ pour l'optimisation)
                m = (V_MAX - V_MIN_MOTEUR) / (V_MAX - V_MIN_NAV2)
                A = (2.0 * (V_MIN_MOTEUR - m * V_MIN_NAV2)) / math.sqrt(V_MIN_NAV2)
                B = 2.0 * m - (V_MIN_MOTEUR / V_MIN_NAV2)

                if abs_smoothed <= V_MIN_NAV2:
                    # Zone 1 : Raccordement concave lisse
                    cmd_speed_hardware = sign * (A * math.sqrt(abs_smoothed) + B * abs_smoothed)
                else:
                    # Zone 2 : Ligne droite vers V_MAX
                    cmd_speed_hardware = sign * (m * (abs_smoothed - V_MIN_NAV2) + V_MIN_MOTEUR)
            else:
                cmd_speed_hardware = 0.0
                
            cmd_speed_hardware = max(min(cmd_speed_hardware, V_MAX), -V_MAX)
            
            # Update state variables
            config['last_angle'] = cmd_angle
            config['last_speed'] = cmd_speed_hardware
            config['last_logical_speed'] = logical_smoothed_speed

            # Conversion and Publication
            rpm_final = (cmd_speed_hardware * 60.0) / (2.0 * math.pi * self.wheel_radius) * config['dir']   
            #rpm_final = 0.0         
            self.wheel_pubs[name].publish(Float64MultiArray(data=[float(rpm_final), float(math.degrees(cmd_angle))]))

            logical_msg = Float64()
            logical_msg.data = float(logical_smoothed_speed)
            self.logical_speed_pubs[name].publish(logical_msg)
            
def main(args=None):
    rclpy.init(args=args)
    node = SwerveKinematics()
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