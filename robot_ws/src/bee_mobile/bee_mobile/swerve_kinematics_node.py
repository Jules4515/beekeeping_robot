#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
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
        self.declare_parameter('limit_motor_speed_rpm', 25.0)
        self.declare_parameter('max_steering_deg', 80.0)
        
        # Software limits (from Nav2 parameters)
        self.declare_parameter('max_linear_speed_ms', 0.23)
        self.declare_parameter('max_angular_speed_rads', 0.36)
        
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
        
        # Chassis geometry (x, y) relative to center, extracted from URDF
        self.wheels = {
            'front_left':  {'x': 0.7661,  'y': 0.5790,  'current_angle': 0.0, 'current_rpm': 0.0, 'last_cmd_angle': 0.0},
            'front_right': {'x': 0.7661,  'y': -0.5790, 'current_angle': 0.0, 'current_rpm': 0.0, 'last_cmd_angle': 0.0},
            'rear_left':   {'x': -0.1939, 'y': 0.5790,  'current_angle': 0.0, 'current_rpm': 0.0, 'last_cmd_angle': 0.0},
            'rear_right':  {'x': -0.1939, 'y': -0.5790, 'current_angle': 0.0, 'current_rpm': 0.0, 'last_cmd_angle': 0.0},
        }

        # --- Odometry State ---
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_time = self.get_clock().now()

        # --- ROS 2 Interfaces ---
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
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

    def encoder_callback(self, msg, wheel_name):
        """ Reads hardware feedback. Assumes msg.data = [RPM, Angle_in_degrees] as floats. """
        if len(msg.data) >= 2:
            self.wheels[wheel_name]['current_rpm'] = float(msg.data[0])
            self.wheels[wheel_name]['current_angle'] = float(msg.data[1])

    def cmd_vel_callback(self, msg):
        """ Inverse Kinematics: Translates global Twist into individual wheel commands """
        
        # --- STEP 1: SOFTWARE LIMITS ---
        # Clamp incoming Nav2 commands to our defined software maximums
        vx = max(-self.max_linear_speed_ms, min(self.max_linear_speed_ms, msg.linear.x))
        vy = max(-self.max_linear_speed_ms, min(self.max_linear_speed_ms, msg.linear.y))
        wz = max(-self.max_angular_speed_rads, min(self.max_angular_speed_rads, msg.angular.z))

        # --- STEP 2: ANTI-STALL LOGIC ---
        # If Nav2 requests a speed lower than the physical minimum of the motors (~10 RPM),
        # we scale up the vectors to reach the minimum speed to prevent stalling/jittering.
        global_speed = math.hypot(vx, vy)
        STOP_TOLERANCE = 0.05
        
        if global_speed < STOP_TOLERANCE and abs(wz) < 0.05:
            # Intentional stop requested by Nav2
            vx, vy, wz = 0.0, 0.0, 0.0
        else:
            # Proportional boost for linear movement to prevent motor stall
            if global_speed < self.min_physical_speed_ms and global_speed >= STOP_TOLERANCE:
                scale_factor = self.min_physical_speed_ms / global_speed
                vx *= scale_factor
                vy *= scale_factor
            
            # Proportional boost for pure rotation to prevent motor stall
            if 0.05 <= abs(wz) < self.min_angular_speed_rads:
                wz = math.copysign(self.min_angular_speed_rads, wz)

        # --- STEP 3: KINEMATICS & HARDWARE CLAMPING ---
        for name, config in self.wheels.items():
            # Local velocity vectors for the wheel
            vx_wheel = vx - config['y'] * wz
            vy_wheel = vy + config['x'] * wz

            # Convert cartesian to polar (speed and angle)
            speed_ms = math.hypot(vx_wheel, vy_wheel)
            
            # Optimization: Prevent wheels from snapping back to 0 degrees when stopped
            if speed_ms > 0.001:
                angle_rad = math.atan2(vy_wheel, vx_wheel)
                config['last_cmd_angle'] = angle_rad
            else:
                angle_rad = config['last_cmd_angle']

            # Format for ESP32 (RPM and Degrees)
            rpm = (speed_ms * 60.0) / (2.0 * math.pi * self.wheel_radius)
            angle_deg = math.degrees(angle_rad)

            # Absolute Hardware Clamping (Protects the mechanical parts against extreme demands)
            rpm = max(-self.limit_motor_speed_rpm, min(self.limit_motor_speed_rpm, rpm))
            angle_deg = max(-self.max_steering_deg, min(self.max_steering_deg, angle_deg))

            # Publish command [RPM, Angle] with explicit float casting
            cmd_msg = Float64MultiArray(data=[float(rpm), float(angle_deg)])
            self.wheel_pubs[name].publish(cmd_msg)

    def odometry_callback(self):
        """ Forward Kinematics: Estimates global robot position from wheel feedback """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        vx_total = 0.0
        vy_total = 0.0
        wz_total = 0.0

        # Sum the kinematic contribution of each wheel
        for config in self.wheels.values():
            speed_ms = (config['current_rpm'] * 2.0 * math.pi * self.wheel_radius) / 60.0
            angle_rad = math.radians(config['current_angle'])

            # Local wheel velocity vectors
            vx_w = speed_ms * math.cos(angle_rad)
            vy_w = speed_ms * math.sin(angle_rad)

            vx_total += vx_w
            vy_total += vy_w
            
            # Angular velocity equation for independent wheels
            r_squared = config['x']**2 + config['y']**2
            wz_total += (config['x'] * vy_w - config['y'] * vx_w) / r_squared

        # Average over the 4 wheels (overdetermined system resolution)
        vx_robot = vx_total / 4.0
        vy_robot = vy_total / 4.0
        wz_robot = wz_total / 4.0

        # Midpoint integration for better accuracy during curves
        delta_theta = wz_robot * dt
        theta_midpoint = self.odom_theta + (delta_theta / 2.0)

        # Rotate local velocities into the global fixed map frame
        delta_x = (vx_robot * math.cos(theta_midpoint) - vy_robot * math.sin(theta_midpoint)) * dt
        delta_y = (vx_robot * math.sin(theta_midpoint) + vy_robot * math.cos(theta_midpoint)) * dt

        # Update absolute position
        self.odom_x += delta_x
        self.odom_y += delta_y
        self.odom_theta += delta_theta

        self.publish_odometry(current_time, vx_robot, vy_robot, wz_robot)

    def publish_odometry(self, current_time, vx, vy, wz):
        """ Packages and publishes the Odometry message and TF tree """
        # Convert Yaw (theta) to Quaternion
        cy = math.cos(self.odom_theta * 0.5)
        sy = math.sin(self.odom_theta * 0.5)

        # 1. Publish TF (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
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
        odom.header.stamp = current_time.to_msg()
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