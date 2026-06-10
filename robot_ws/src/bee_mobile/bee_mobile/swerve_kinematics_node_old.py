#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Int8
from tf2_ros import TransformBroadcaster
import math

class SwerveKinematicsNode(Node):
    """
    Handles continuous inverse and forward kinematics for a 4WS (Swerve/Omni) robot.
    Subscribes to /cmd_vel (Nav2).
    Publishes and subscribes to /mobile/wheel_*/... for ESP32 hardware (Inverse Kinematics).
    Computes and publishes /odom and TF (Forward Kinematics).
    """
    def __init__(self):
        super().__init__('swerve_kinematics_node')

        # --- Robot Physical Parameters ---
        self.wheel_radius = 0.215  # 0.43m diameter / 2
        
        # --- Safety Limits (Software & Hardware) ---
        # Hardware absolute limits (Priority 1)
        self.declare_parameter('limit_motor_speed_rpm', 30.0)
        self.declare_parameter('max_steering_deg', 80.0)
        
        # Limit the physical speed of the steering mechanism to prevent mechanical shock
        self.declare_parameter('max_steering_speed_deg_s', 15.0) 
        self.max_steering_speed_rad_s = math.radians(self.get_parameter('max_steering_speed_deg_s').value)
        self.last_cmd_time = self.get_clock().now()

        # --- Watchdog Parameters ---
        self.cmd_timeout_sec = 0.5  # Trigger safety stop after 0.5 seconds of silence
        self.watchdog_triggered = False # Prevents spamming stop commands on the network

        # Software limits (from Nav2 parameters)
        self.declare_parameter('max_linear_speed_ms', 0.67)
        self.declare_parameter('max_angular_speed_rads', 0.71)
        self.declare_parameter('max_accel_ms2', 0.5) 
        
        # Anti-stall parameters (~10 RPM threshold to prevent motor jitter)
        self.declare_parameter('min_physical_speed_ms', 0.23)
        self.declare_parameter('min_angular_speed_rads', 0.35) 

        # Fetch parameters
        self.limit_motor_speed_rpm = self.get_parameter('limit_motor_speed_rpm').value
        self.max_steering_deg = self.get_parameter('max_steering_deg').value
        self.max_linear_speed_ms = self.get_parameter('max_linear_speed_ms').value
        self.max_angular_speed_rads = self.get_parameter('max_angular_speed_rads').value
        self.min_physical_speed_ms = self.get_parameter('min_physical_speed_ms').value
        self.min_angular_speed_rads = self.get_parameter('min_angular_speed_rads').value
        self.max_accel_ms2 = self.get_parameter('max_accel_ms2').value
        
        # Chassis geometry (x, y) relative to center, extracted from URDF + Hardware direction (dir)
        # 'dir' controls the command sent to ESP32.
        # 'enc_dir' corrects the raw feedback received from ESP32.
        self.wheels = {
            'front_left':  {'x': 0.48,  'y': 0.4150,  'dir': 1.0, 'enc_dir': 1.0,  'current_angle': 0.0, 'current_rpm': 0.0, 'last_cmd_angle': 0.0, 'last_cmd_speed_ms': 0.0},
            'front_right': {'x': 0.48,  'y': -0.4150, 'dir': -1.0, 'enc_dir': -1.0, 'current_angle': 0.0, 'current_rpm': 0.0, 'last_cmd_angle': 0.0, 'last_cmd_speed_ms': 0.0},
            'rear_left':   {'x': -0.48, 'y': 0.4150,  'dir': 1.0, 'enc_dir': 1.0,  'current_angle': 0.0, 'current_rpm': 0.0, 'last_cmd_angle': 0.0, 'last_cmd_speed_ms': 0.0},
            'rear_right':  {'x': -0.48, 'y': -0.4150, 'dir': -1.0, 'enc_dir': -1.0, 'current_angle': 0.0, 'current_rpm': 0.0, 'last_cmd_angle': 0.0, 'last_cmd_speed_ms': 0.0},
        }

        # --- Odometry State ---
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_time = self.get_clock().now()

        # Filtrage exp
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_wz = 0.0
        
        self.vxy_alpha = 0.2  # Coefficient de lissage (0.2 = modéré)
        self.wz_alpha = 0.1  # Coefficient de lissage (0.1 = très lissé)

        # --- Hardware State Memory ---
        self.active_joy_mode = -1

        # --- ROS 2 Interfaces ---
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_out', self.cmd_vel_callback, 10)
        self.mode_sub = self.create_subscription(Int8, '/joystick_control_mode', self.mode_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.wheel_pubs = {}

        # Dynamically create publishers and subscribers for all 4 wheels
        for name in self.wheels.keys():
            pub_topic = f'mobile/wheel_{name}/motor_speed'
            self.wheel_pubs[name] = self.create_publisher(Float64MultiArray, pub_topic, 10)
            
            sub_topic = f'mobile/wheel_{name}/encoder_angle'
            self.create_subscription(
                Float64MultiArray, 
                sub_topic, 
                lambda msg, n=name: self.encoder_callback(msg, n), 
                10
            )

        # Main odometry loop (50Hz = 0.02s)
        self.odom_timer = self.create_timer(0.02, self.odometry_callback)
        self.get_logger().info("Swerve Kinematics Node Initialized with Anti-Stall (0.23m/s) and Limits.")

    def mode_callback(self, msg):
        """ Asynchronously updates the hardware interaction state (-1 means deadman off) """
        self.active_joy_mode = msg.data

    def encoder_callback(self, msg, wheel_name):
        """ Reads hardware feedback and corrects asymmetric physical mounting signs. """
        if len(msg.data) >= 2:
            # Apply enc_dir to fix the raw RPM feedback before odometry calculation
            self.wheels[wheel_name]['current_rpm'] = float(msg.data[0]) * self.wheels[wheel_name]['enc_dir']
            self.wheels[wheel_name]['current_angle'] = float(msg.data[1])

    def cmd_vel_callback(self, msg):
        """ Inverse Kinematics: Translates global Twist into individual wheel commands """
        
        # Reset the watchdog flag since a new command just arrived
        self.watchdog_triggered = False

        # --- TIME DELTA CALCULATION ---
        current_time = self.get_clock().now()
        dt = (current_time - self.last_cmd_time).nanoseconds / 1e9
        self.last_cmd_time = current_time
        
        if dt <= 0.0 or dt > 0.5:
            dt = 0.02

        # --- 1. DETERMINE GLOBAL STATE ---
        is_idle = (abs(msg.linear.x) < 0.005 and abs(msg.linear.y) < 0.005 and abs(msg.angular.z) < 0.005)

        if not is_idle:
            vx = max(-self.max_linear_speed_ms, min(self.max_linear_speed_ms, msg.linear.x))
            vy = max(-self.max_linear_speed_ms, min(self.max_linear_speed_ms, msg.linear.y))
            wz = max(-self.max_angular_speed_rads, min(self.max_angular_speed_rads, msg.angular.z))
        
        for name, config in self.wheels.items():
            last_angle_rad = config['last_cmd_angle']
            
            # --- 2. TARGET GENERATION (Branching Logic) ---
            if is_idle:
                # Robot is resting: Apply pre-orientation modes
                raw_target_speed_ms = 0.0
                if self.active_joy_mode == 3:
                    raw_target_angle = math.atan2(config['x'], -(config['y']))
                elif self.active_joy_mode in [1, 2]:
                    raw_target_angle = 0.0
                else:
                    raw_target_angle = last_angle_rad
            else:
                # Robot is moving: Apply Inverse Kinematics
                vx_wheel = vx - config['y'] * wz
                vy_wheel = vy + config['x'] * wz
                raw_target_speed_ms = math.hypot(vx_wheel, vy_wheel)

                if raw_target_speed_ms > 0.001:
                    raw_target_angle = math.atan2(vy_wheel, vx_wheel)
                else:
                    raw_target_angle = last_angle_rad
                    raw_target_speed_ms = 0.0

            # --- 3. UNIFIED PROCESSING PIPELINE ---
            
            # 3.1 Vectorial Inversion Logic (Applies to both modes and movement)
            if abs(raw_target_angle) > (math.pi / 2.0):
                if raw_target_angle > 0.0:
                    target_angle_rad = raw_target_angle - math.pi
                else:
                    target_angle_rad = raw_target_angle + math.pi
                raw_target_speed_ms = -raw_target_speed_ms
            else:
                target_angle_rad = raw_target_angle

            # 3.2 Slew Rate Limiter
            step_diff = (target_angle_rad - last_angle_rad + math.pi) % (2.0 * math.pi) - math.pi
            max_step = self.max_steering_speed_rad_s * dt
            
            limited_step = max(-max_step, min(max_step, step_diff))
            final_angle_rad = (last_angle_rad + limited_step + math.pi) % (2.0 * math.pi) - math.pi

            # 3.3 Hardware Clamping
            final_angle_deg = math.degrees(final_angle_rad)
            clamped_angle_deg = max(-self.max_steering_deg, min(self.max_steering_deg, final_angle_deg))
            config['last_cmd_angle'] = math.radians(clamped_angle_deg)

            # 3.4 Traction Scaling & Publishing
            if is_idle or raw_target_speed_ms == 0.0:
                final_speed_ms = 0.0
            else:
                # CRITIQUE : Comparer avec l'angle PHYSIQUE actuel renvoyé par l'encodeur, 
                # et non l'angle commandé (config['last_cmd_angle'])
                current_angle_rad = math.radians(config['current_angle'])
                alignment_error = abs((target_angle_rad - current_angle_rad + math.pi) % (2.0 * math.pi) - math.pi)

                # TOLÉRANCE : Si la roue est à plus de 15° de sa cible, on coupe la traction.
                tolerance_rad = math.radians(10.0) 
                
                if alignment_error > tolerance_rad:
                    final_speed_ms = 0.0  # La roue pivote sur place sans avancer
                else:
                    # Dans les 10° finaux, on applique la traction lissée par le cosinus
                    final_speed_ms = raw_target_speed_ms * math.cos(alignment_error)

            # Apply anti-stall minimum physical speed threshold
            # If a small non-zero command is requested, bump it to the
            # minimum physical speed (preserving sign) so the robot can start.
            if final_speed_ms != 0.0 and abs(final_speed_ms) < self.min_physical_speed_ms:
                final_speed_ms = math.copysign(self.min_physical_speed_ms, final_speed_ms)

            # ---------------------------------------------------------
            # Software Slew Rate Limiter
            # WHY: Prevents wheel slip on the 4WS heavy base by capping acceleration,
            # ensuring the PID controller receives a trackable ramp instead of a raw step.
            # ---------------------------------------------------------
            
            # Calculate max allowable velocity delta for the current physical time step
            max_speed_step = self.max_accel_ms2 * dt
            
            # Determine the raw velocity gap
            speed_diff = final_speed_ms - config['last_cmd_speed_ms']
            
            # Clamp the requested acceleration to the physical capabilities of the robot
            limited_speed_step = max(-max_speed_step, min(max_speed_step, speed_diff))
            
            # Integrate the clamped step into the velocity profile
            ramped_speed_ms = config['last_cmd_speed_ms'] + limited_speed_step
            
            # Persist kinematic state for the next control loop
            config['last_cmd_speed_ms'] = ramped_speed_ms

            # Convert the dynamically smoothed velocity into motor RPM
            rpm = (ramped_speed_ms * 60.0) / (2.0 * math.pi * self.wheel_radius) * config['dir']
            rpm = max(-self.limit_motor_speed_rpm, min(self.limit_motor_speed_rpm, rpm))

            self.wheel_pubs[name].publish(Float64MultiArray(data=[float(rpm), float(clamped_angle_deg)]))

    def odometry_callback(self):
        """ Forward Kinematics: Estimates global robot position from wheel feedback using CoR """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        # --- 0. WATCHDOG SOFTWARE ---
        # Calculate how much time passed without receiving a velocity command
        dt_watchdog = (current_time - self.last_cmd_time).nanoseconds / 1e9
        
        if dt_watchdog > self.cmd_timeout_sec:
            if not self.watchdog_triggered:
                self.get_logger().warn("Watchdog triggered: No cmd_vel received for 0.5s. Forcing motors to 0 RPM.")
                
                # Override physical commands to stop the robot
                for name, config in self.wheels.items():
                    # Send 0.0 RPM but keep the last commanded steering angle
                    clamped_angle_deg = math.degrees(config['last_cmd_angle'])
                    self.wheel_pubs[name].publish(Float64MultiArray(data=[0.0, float(clamped_angle_deg)]))
                
                # Lock the watchdog so it only publishes the stop command once per timeout event
                self.watchdog_triggered = True
        
        # 1. Calculate individual wheel velocity vectors in the robot frame
        vx_sum = 0.0
        vy_sum = 0.0

        for config in self.wheels.values():
            speed_ms = (config['current_rpm'] * 2.0 * math.pi * self.wheel_radius) / 60.0
            angle_rad = math.radians(config['current_angle'])

            config['vx_w'] = speed_ms * math.cos(angle_rad)
            config['vy_w'] = speed_ms * math.sin(angle_rad)
            
            vx_sum += config['vx_w']
            vy_sum += config['vy_w']

        # 2. Linear Velocity of the Geometric Center (CoR)
        vx_cor = vx_sum / 4.0
        vy_cor = vy_sum / 4.0

        # 3. Exact Angular Velocity (wz) using Least Squares relative to CoR
        wz_numerator = 0.0
        wz_denominator = 0.0
        for config in self.wheels.values():
            # Distance from CoR
            x_rel = config['x']
            y_rel = config['y']
            
            wz_numerator += x_rel * (config['vy_w'] - vy_cor) - y_rel * (config['vx_w'] - vx_cor)
            wz_denominator += x_rel**2 + y_rel**2

        wz_robot = wz_numerator / wz_denominator if wz_denominator > 0 else 0.0

        # 4. Transfer velocity from CoR back to base_footprint (for Nav2 and RViz)
        # V_base = V_cor + (Omega x R_cor_to_base)
        vx_robot = vx_cor
        vy_robot = vy_cor 

        # 5. Odometry Integration (Position update)
        delta_theta = wz_robot * dt
        theta_midpoint = self.odom_theta + (delta_theta / 2.0)

        delta_x = (vx_robot * math.cos(theta_midpoint) - vy_robot * math.sin(theta_midpoint)) * dt
        delta_y = (vx_robot * math.sin(theta_midpoint) + vy_robot * math.cos(theta_midpoint)) * dt

        self.odom_x += delta_x
        self.odom_y += delta_y
        self.odom_theta += delta_theta
        
        # Filtrage exp vx vy wz
        self.filtered_vx = self.vxy_alpha * vx_robot + (1 - self.vxy_alpha) * self.filtered_vx
        self.filtered_vy = self.vxy_alpha * vy_robot + (1 - self.vxy_alpha) * self.filtered_vy
        self.filtered_wz = self.wz_alpha * wz_robot + (1 - self.wz_alpha) * self.filtered_wz

        self.publish_odometry(current_time, self.filtered_vx, self.filtered_vy, self.filtered_wz)

    def publish_odometry(self, current_time, vx, vy, wz):
        """ Packages and publishes the Odometry message and TF tree """
        # Convert Yaw (theta) to Quaternion
        cy = math.cos(self.odom_theta * 0.5)
        sy = math.sin(self.odom_theta * 0.5)

        # ---------------------------------------------------------
        # TF Forward-Dating (Extrapolation Buffer)
        # WHY: Adds 50ms to the TF stamp so asynchronous nodes (like AMCL or RViz) 
        # asking for 'now()' don't hit the "extrapolation into the future" exception.
        # ---------------------------------------------------------
        tf_time = current_time + rclpy.duration.Duration(seconds=0.10)

        # 1. Publish TF (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = tf_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish Odometry Message
        odom = Odometry()
        odom.header.stamp = tf_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        
        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()