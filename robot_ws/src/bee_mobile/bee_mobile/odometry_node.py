#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class FourWSOdometry(Node):
    def __init__(self):
        super().__init__('four_ws_odometry')

        # --- HARDCODED CALIBRATION SWITCH ---
        # Set to True to apply the offsets below. 
        # Set to False once your motors/encoders are physically or firmware-calibrated.
        self.use_calibration = False 

        # --- Physical Robot Parameters ---
        self.declare_parameter('wheel_diameter_m', 0.43)
        self.wheel_diameter = self.get_parameter('wheel_diameter_m').value
        self.wheel_circumference = math.pi * self.wheel_diameter

        # Wheel positions from URDF (Origin is shifted towards the rear)
        self.wheel_positions = {
            'front_left':  (0.7661,  0.5790),
            'front_right': (0.7661, -0.5790),
            'rear_left':   (-0.1939,  0.5790),
            'rear_right':  (-0.1939, -0.5790),
        }
        self.wheel_names = list(self.wheel_positions.keys())
        
        # --- Kinematic Polarity ---
        # Right side motors are physically mirrored. 
        # We invert them so forward motion translates to positive velocity for the solver.
        self.rpm_polarity = {
            'front_left':   1.0,
            'front_right': -1.0, 
            'rear_left':    1.0,
            'rear_right':  -1.0,
        }
        
        # --- Calibration Offsets (Zero-centering) ---
        # Based on your rqt_plot analysis: offsets to center the Neutral/Zero point.
        self.rpm_offset = {
            'front_left':   6.5,
            'front_right': -3.5, 
            'rear_left':    3.5,
            'rear_right':  -3.5,
        }

        # Latest raw data from subscribers
        self.wheel_data = {name: [0.0, 0.0] for name in self.wheel_names}

        # --- ROS Communications ---
        for name in self.wheel_names:
            topic = f'/mobile/wheel_{name}/encoder_angle'
            self.create_subscription(
                Float64MultiArray, 
                topic, 
                lambda msg, n=name: self.encoder_callback(msg, n), 
                10
            )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Odometry Pose Accumulators ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()
        self.create_timer(0.02, self.update_odometry) # 50Hz update cycle

        self.get_logger().info(f'4WS Odom Node: Calibration is {"ENABLED" if self.use_calibration else "DISABLED"} in code.')

    def encoder_callback(self, msg: Float64MultiArray, wheel_name: str):
        if len(msg.data) >= 2:
            self.wheel_data[wheel_name] = [float(msg.data[0]), float(msg.data[1])]

    def compute_chassis_twist(self) -> tuple[float, float, float]:
        """
        Calculates [vx, vy, omega_z] using Least Squares.
        Handles the deadband, calibration offsets, and polarity logic.
        """
        A = np.zeros((4, 3))
        B = np.zeros(4)
        
        # Counter to track if the robot is physically stopped
        stopped_wheels = 0

        for i, name in enumerate(self.wheel_names):
            raw_rpm, raw_steer = self.wheel_data[name]
            
            # 1. Zero-Clamp (Anti-Ghosting)
            # If raw input is within sensor noise (<= 3 RPM), force absolute zero.
            # This prevents offsets from creating fake movement when joystick is released.
            if abs(raw_rpm) <= 3.0: 
                rpm = 0.0
                stopped_wheels += 1
            else:
                # 2. Apply Calibration and Polarity
                if self.use_calibration:
                    rpm = (raw_rpm + self.rpm_offset[name]) * self.rpm_polarity[name]
                else:
                    rpm = raw_rpm * self.rpm_polarity[name]

            # 3. Calculate Wheel Linear Velocity
            v_i = (rpm / 60.0) * self.wheel_circumference
            
            # Convert steering to radians
            theta_i = math.radians(raw_steer)
            x_i, y_i = self.wheel_positions[name]

            # 4. Populate Matrices for Least Squares Solver
            # Equation: cos(th)*vx + sin(th)*vy + (xi*sin(th) - yi*cos(th))*wz = vi
            A[i, 0] = math.cos(theta_i)
            A[i, 1] = math.sin(theta_i)
            A[i, 2] = x_i * math.sin(theta_i) - y_i * math.cos(theta_i) 
            B[i] = v_i

        # If all 4 wheels are clamped to zero, return zero twist immediately
        if stopped_wheels == 4:
            return 0.0, 0.0, 0.0

        try:
            # Solve the system. rcond filters out noise in the matrix rank.
            solution, residuals, rank, s = np.linalg.lstsq(A, B, rcond=0.05)
            
            v_x = float(solution[0])
            v_y = float(solution[1])
            omega_z = float(solution[2])
            self.rpm_offset = {
            'front_left':   6.0,   # Règle le décalage de la roue bleue
            'front_right': -2.5,   # Règle le décalage de la roue verte (approx)
            'rear_left':    2.0,   # Règle le décalage de la roue rose
            'rear_right':  -2.5,   # Règle le décalage de la roue cyan (approx)
        }
            # Final safety clamps on estimated chassis speeds
            v_x = max(-3.0, min(3.0, v_x))
            v_y = max(-3.0, min(3.0, v_y))
            omega_z = max(-2.0, min(2.0, omega_z))
            
            return v_x, v_y, omega_z
            
        except Exception as e:
            self.get_logger().warn(f'Kinematics Error: {e}')
            return 0.0, 0.0, 0.0

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = now

        if dt <= 0.0 or dt > 0.5:
            return

        v_x, v_y, omega_z = self.compute_chassis_twist()

        # Integration using Exact Circular Arc formulas
        if abs(omega_z) < 1e-4:
            # Linear Motion
            self.x += (v_x * math.cos(self.theta) - v_y * math.sin(self.theta)) * dt
            self.y += (v_x * math.sin(self.theta) + v_y * math.cos(self.theta)) * dt
        else:
            # Circular Arc Motion
            delta_theta = omega_z * dt
            self.x += (v_x / omega_z) * (math.sin(self.theta + delta_theta) - math.sin(self.theta)) - \
                      (v_y / omega_z) * (math.cos(self.theta + delta_theta) - math.cos(self.theta))
            self.y += (v_x / omega_z) * (math.cos(self.theta) - math.cos(self.theta + delta_theta)) + \
                      (v_y / omega_z) * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.theta += delta_theta

        # Normalize heading to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Euler to Quaternion for ROS
        q_z = math.sin(self.theta / 2.0)
        q_w = math.cos(self.theta / 2.0)

        # --- Publish Odom ---
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.linear.y = v_y
        odom.twist.twist.angular.z = omega_z
        self.odom_pub.publish(odom)

        # --- Publish TF ---
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = q_z
        tf.transform.rotation.w = q_w
        self.tf_broadcaster.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    node = FourWSOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()