#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Float64, Int8
import math

class SwerveKinematics(Node):
    def __init__(self):
        super().__init__('swerve_kinematics')
        
        # --- 1. INITIALISATION DES VARIABLES & PARAMÈTRES ---
        self._init_parameters()
        self._init_kinematic_constants()
        self._init_wheel_hardware()
        self._init_ros_interfaces()

    def _init_parameters(self):
        """Loads and converts ROS 2 parameters to class variables."""
        self.declare_parameter('max_steer_deg_s', 90.0)
        self.declare_parameter('hard_limit_deg', 80.0)
        self.declare_parameter('v_max_physical', 0.70)
        self.declare_parameter('v_min_moteur', 0.30)
        self.declare_parameter('v_min_nav2', 0.10)
        self.declare_parameter('deadband_vx', 0.05)
        self.declare_parameter('deadband_wz', 0.05)
        self.declare_parameter('opposite_max_angle_deg', 50.0)

        # Tolérance d'alignement
        self.declare_parameter('enable_align_tolerance', False)
        self.declare_parameter('align_tolerance_deg', 5.0)
        
        # Paramètres du Slew Rate Linéaire
        self.declare_parameter('enable_speed_slew_rate', True)
        self.declare_parameter('accel_max', 2.0)
        self.declare_parameter('decel_max', 2.0)

        self.max_steer_rad_s = math.radians(self.get_parameter('max_steer_deg_s').value)
        self.hard_limit_rad = math.radians(self.get_parameter('hard_limit_deg').value)
        self.v_max_physical = self.get_parameter('v_max_physical').value
        self.v_min_moteur = self.get_parameter('v_min_moteur').value
        self.v_min_nav2 = self.get_parameter('v_min_nav2').value
        self.deadband_vx = self.get_parameter('deadband_vx').value
        self.deadband_wz = self.get_parameter('deadband_wz').value
        self.opposite_max_angle_rad = math.radians(self.get_parameter('opposite_max_angle_deg').value)

        self.enable_align_tolerance = self.get_parameter('enable_align_tolerance').value
        self.align_tolerance_rad = math.radians(self.get_parameter('align_tolerance_deg').value)
        
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
            'rear_left':   {'x': -0.48, 'y': 0.4150,  'dir': 1.0, 'enc_dir': 1.0,  'current_angle': 0.0, 'current_rpm': 0.0, 'last_angle': 0.0, 'last_target_angle': 0.0, 'last_logical_speed': 0.0},
            'rear_right':  {'x': -0.48, 'y': -0.4150, 'dir': 1.0, 'enc_dir': -1.0, 'current_angle': 0.0, 'current_rpm': 0.0, 'last_angle': 0.0, 'last_target_angle': 0.0, 'last_logical_speed': 0.0},
        }
        self.L_half = 0.48
        self.W_half = 0.4150

        # Pré-calcul du Rayon CIR minimal absolu
        self.R_min = (self.L_half / math.tan(self.opposite_max_angle_rad)) + self.W_half

    def _init_ros_interfaces(self):
        """Sets up ROS 2 publishers, subscribers, and timers."""
        self.active_joy_mode = -1
        self.last_cmd_time = self.get_clock().now()

        self.mode_sub = self.create_subscription(Int8, '/joystick_control_mode', self.mode_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_out', self.cmd_vel_callback, 10)
        
        self.wheel_pubs = {}

        for name in self.wheels.keys():
            self.wheel_pubs[name] = self.create_publisher(Float64MultiArray, f'mobile/wheel_{name}/motor_speed', 10)
            self.create_subscription(Float64MultiArray, f'mobile/wheel_{name}/encoder_angle', 
                                     lambda msg, n=name: self.encoder_callback(msg, n), 10)

    # --- 2. CALLBACKS ---

    def mode_callback(self, msg):
        self.active_joy_mode = msg.data

    def encoder_callback(self, msg, wheel_name):
        if len(msg.data) >= 2:
            self.wheels[wheel_name]['current_rpm'] = float(msg.data[0]) * self.wheels[wheel_name]['enc_dir']
            self.wheels[wheel_name]['current_angle'] = float(msg.data[1])

    def cmd_vel_callback(self, msg):
        """Main kinematic pipeline executed on every Nav2 command."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_cmd_time).nanoseconds / 1e9
        self.last_cmd_time = current_time
        
        if dt <= 0.0 or dt > 0.5:
            dt = 0.1

        current_state, vx, wz, cir_radius = self._compute_fsm_state(msg.linear.x, msg.angular.z)
        
        # Affichage Console à 10 Hz
        self.get_logger().info(f"Mode: {current_state} | CIR Radius: {cir_radius:.3f} m")

        self._calculate_wheel_targets(current_state, vx, wz, dt)
        is_aligned = self._check_alignment(current_state)
        
        # Passage du dt pour le calcul du slew rate de vitesse
        self._publish_hardware_commands(is_aligned, dt)

    # --- 3. MÉTHODES CINÉMATIQUES INTERNES ---

    def _compute_fsm_state(self, vx, wz):
        """Determines kinematic mode, applies Ackermann constraints, and calculates CIR radius."""
        is_vx_active = abs(vx) > self.deadband_vx
        is_wz_active = abs(wz) > self.deadband_wz

        if is_wz_active and not is_vx_active:
            current_state = 'PIVOT'
            self.get_logger().info("State => PIVOT")
            vx = 0.0
            cir_radius = 0.0
        elif is_vx_active and not is_wz_active:
            current_state = 'STRAIGHT'
            self.get_logger().info("State => STRAIGHT")
            wz = 0.0
            cir_radius = float('inf')
        elif is_vx_active and is_wz_active:
            current_state = 'OPPOSITE'
            self.get_logger().info("State => OPPOSITE")
        else:
            current_state = 'STOP'
            self.get_logger().info("State => STOP")
            vx = 0.0
            wz = 0.0
            cir_radius = float('inf')

        # Application de la saturation géométrique
        if current_state == 'OPPOSITE':
            R = vx / wz
            if abs(R) < self.R_min:
                clamped_R = math.copysign(self.R_min, R)
                wz = vx / clamped_R # Bridage de la rotation pour respecter l'angle max
                cir_radius = abs(clamped_R)
            else:
                cir_radius = abs(R)

        return current_state, vx, wz, cir_radius

    def _calculate_wheel_targets(self, current_state, vx, wz, dt):
        """Computes ideal angles/speeds, applies phase inversion, and enforces steer slew rate."""
        for name, config in self.wheels.items():
            if current_state == 'STOP':
                raw_speed = 0.0
                if self.active_joy_mode == 3:
                    raw_angle = math.atan2(config['x'], -config['y'])
                elif self.active_joy_mode in [1, 2]:
                    raw_angle = 0.0
                else:
                    raw_angle = config['last_target_angle']
            else:
                vx_w = vx - config['y'] * wz
                vy_w = config['x'] * wz 
                raw_speed = math.hypot(vx_w, vy_w)
                raw_angle = math.atan2(vy_w, vx_w)

            # Phase Inversion for all states
            if raw_angle > (math.pi / 2.0):
                ideal_angle = raw_angle - math.pi
                ideal_speed = -raw_speed
            elif raw_angle < -(math.pi / 2.0):
                ideal_angle = raw_angle + math.pi
                ideal_speed = -raw_speed
            else:
                ideal_angle = raw_angle
                ideal_speed = raw_speed
                    
            config['last_target_angle'] = ideal_angle

            # Steer Slew Rate Limiter
            angle_diff = ideal_angle - config['last_angle']
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            max_step_angle = self.max_steer_rad_s * dt

            if angle_diff > max_step_angle:
                cmd_angle = config['last_angle'] + max_step_angle
            elif angle_diff < -max_step_angle:
                cmd_angle = config['last_angle'] - max_step_angle
            else:
                cmd_angle = ideal_angle

            cmd_angle = max(min(cmd_angle, self.hard_limit_rad), -self.hard_limit_rad)
            
            config['temp_cmd_angle'] = cmd_angle
            config['temp_ideal_speed'] = ideal_speed

    def _check_alignment(self, current_state):
        """Applies dynamic angular tolerance based on kinematic state."""
        if not self.enable_align_tolerance or current_state == 'STOP':
            return True

        # Tolérance stricte (ex: 2°) pour le pivot sur place
        if current_state == 'PIVOT':
            tolerance = self.align_tolerance_rad
        # Tolérance souple (ex: 15°) en plein mouvement pour absorber le retard mécanique (Tracking Lag)
        else:
            tolerance = math.radians(15.0) 

        for name, config in self.wheels.items():
            phys_angle = math.radians(config['current_angle'])
            target = config['temp_cmd_angle']
            err = target - phys_angle
            err = abs(math.atan2(math.sin(err), math.cos(err)))
            
            # Le blocage d'urgence ne s'active que si l'erreur dépasse la tolérance du mode actif
            if err > tolerance:
                return False
                
        return True

    def _publish_hardware_commands(self, is_aligned, dt):
        """Applies linear slew rate, non-linear speed mapping, and publishes commands."""
        precalculated_commands = {}

        for name, config in self.wheels.items():
            cmd_angle = config['temp_cmd_angle']
            
            if not is_aligned:
                ideal_speed = 0.0
            else:
                ideal_speed = config['temp_ideal_speed']

            logical_target_speed = max(min(ideal_speed, self.v_max_physical), -self.v_max_physical)

            # ============================================================
            # Linar Speed Slew Rate Limiter
            # ============================================================
            if self.enable_speed_slew_rate:
                last_logical_spd = config['last_logical_speed']
                speed_diff = logical_target_speed - last_logical_spd
                
                is_accelerating = abs(logical_target_speed) > abs(last_logical_spd) or (logical_target_speed * last_logical_spd < 0)
                max_step_lin = (self.accel_max if is_accelerating else self.decel_max) * dt
                    
                if abs(speed_diff) > max_step_lin:
                    logical_smoothed_speed = last_logical_spd + math.copysign(max_step_lin, speed_diff)
                else:
                    logical_smoothed_speed = logical_target_speed
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
            print(Float64MultiArray(data=[float(cmds['rpm']), float(cmds['angle_deg'])]))
            
def main(args=None):
    rclpy.init(args=args)
    node = SwerveKinematics()
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