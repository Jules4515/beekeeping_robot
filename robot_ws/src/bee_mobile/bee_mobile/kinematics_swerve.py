#!/usr/bin/env python3
# 
# Swerve Kinematics Controller for ROS2
#
# This node converts commanded linear and angular velocities into steering
# angles and wheel speeds for a four-wheel swerve drive. It applies deadband
# filtering, kinematic mode transitions, steering alignment checks, geometric
# limits, slew-rate limiting, nonlinear motor-speed mapping, and publishes
# commands to the wheel controllers.
# 

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Int8
import math

class KinematicsSwerve(Node):
    def __init__(self):
        super().__init__('kinematics_swerve')
        
        # --- 1. VARIABLE AND PARAMETER INITIALIZATION ---
        self._init_parameters()
        self._init_kinematic_constants()
        self._init_wheel_hardware()
        self._init_ros_interfaces()

    def _init_parameters(self):
        """Loads and converts ROS 2 parameters to class variables."""
        self.declare_parameter('max_steer_deg_s', 70.0)
        self.declare_parameter('hard_limit_deg', 50.0)
        self.declare_parameter('v_max_physical', 0.75)
        self.declare_parameter('v_min_moteur', 0.40)
        self.declare_parameter('v_min_nav2', 0.10)
        self.declare_parameter('deadband_vx', 0.05)
        self.declare_parameter('deadband_wz', 0.05)
        self.declare_parameter('opposite_max_angle_deg', 40.0)

        # Alignment tolerance.
        self.declare_parameter('align_tolerance_deg', 7.0)
        
        # Linear slew-rate parameters.
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

        # Precompute the absolute minimum CIR radius.
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
            
        self.current_mode = 'STOP'
        self.waiting_for_alignment = False

    # --- 2. CALLBACKS ---

    def mode_callback(self, msg):
        self.active_joy_mode = msg.data

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

        new_state, vx, wz, cir_radius = self._compute_fsm_state(msg.linear.x, msg.angular.z)
        
        # --- TRANSITION LOGIC ---
        if new_state != self.current_mode:
            if new_state == 'STOP':
                self.waiting_for_alignment = False # No alignment required to stop.
            elif self.current_mode in ['STRAIGHT', 'OPPOSITE'] and new_state in ['STRAIGHT', 'OPPOSITE']:
                pass # Smooth transition: maintain forward motion.
            else:
                self.waiting_for_alignment = True # PIVOT <-> motion or STOP -> motion.
                
            self.current_mode = new_state

        self.get_logger().info(f"Mode: {self.current_mode} | CIR: {cir_radius:.3f}m | Aligning: {self.waiting_for_alignment}")

        self._calculate_wheel_targets(self.current_mode, vx, wz, dt)
        is_aligned = self._check_alignment()
        self._publish_hardware_commands(is_aligned, dt)

    # --- 3. INTERNAL KINEMATIC METHODS ---

    def _compute_fsm_state(self, vx, wz):
        """Determines kinematic mode, applies Ackermann constraints, and calculates CIR radius."""
        is_vx_active = abs(vx) > self.deadband_vx
        is_wz_active = abs(wz) > self.deadband_wz

        if is_wz_active and not is_vx_active:
            current_state = 'PIVOT'
            vx = 0.0
            cir_radius = 0.0
        elif is_vx_active and not is_wz_active:
            current_state = 'STRAIGHT'
            wz = 0.0
            cir_radius = float('inf')
        elif is_vx_active and is_wz_active:
            current_state = 'OPPOSITE'
        else:
            current_state = 'STOP'
            vx = 0.0
            wz = 0.0
            cir_radius = float('inf')

        # Apply geometric saturation.
        if current_state == 'OPPOSITE':
            R = vx / wz
            if abs(R) < self.R_min:
                clamped_R = math.copysign(self.R_min, R)
                wz = vx / clamped_R # Clamp rotation to respect the maximum angle.
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
                elif self.active_joy_mode == 1:
                    raw_angle = 0.0
                else:
                    raw_angle = config['last_target_angle']
            else:
                vx_w = vx - config['y'] * wz
                vy_w = config['x'] * wz 
                raw_speed = math.hypot(vx_w, vy_w)
                raw_angle = math.atan2(vy_w, vx_w)

            # Phase inversion for all states.
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

            
    def _check_alignment(self):
        """Verifies strict alignment only when the wait flag is triggered by a major transition."""
            
        # Allow traction when no major transition has locked the system.
        if not self.waiting_for_alignment:
            return True

        is_aligned = True

        # Strict verification mode (Stop & Steer).
        for name, config in self.wheels.items():
            phys_angle = math.radians(config['current_angle'])
            target = config['last_target_angle'] 
            err = target - phys_angle
            err = abs(math.atan2(math.sin(err), math.cos(err)))
            
            if err > self.align_tolerance_rad:
                is_aligned = False # Not aligned yet; block forward motion.
        
        if not is_aligned:
            return False
        else:
            # If the loop completes without returning False, all four wheels are aligned.
            self.waiting_for_alignment = False
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
            # Linear speed slew-rate limiter.
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
            
            # Store the command in the local dictionary.
            precalculated_commands[name] = {
                'rpm': rpm_final,
                'angle_deg': math.degrees(cmd_angle),
            }

        for name, cmds in precalculated_commands.items():
            self.wheel_pubs[name].publish(Float64MultiArray(data=[float(cmds['rpm']), float(cmds['angle_deg'])]))
            
def main(args=None):
    rclpy.init(args=args)
    node = KinematicsSwerve()
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