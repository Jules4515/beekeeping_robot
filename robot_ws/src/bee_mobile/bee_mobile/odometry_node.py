#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class FourWSOdometry(Node):
    def __init__(self):
        super().__init__('four_ws_odometry')

        # Parameters (match your robot)
        self.declare_parameter('wheel_diameter_m', 0.43)
        self.declare_parameter('wheel_base_m', 0.96)
        self.declare_parameter('wheel_separation_m', 0.83)
        self.declare_parameter('encoder_ppr', 102400)  # pulses per rev

        self.wheel_diam = self.get_parameter('wheel_diameter_m').value
        self.wheel_base = self.get_parameter('wheel_base_m').value
        self.wheel_sep = self.get_parameter('wheel_separation_m').value
        self.encoder_ppr = self.get_parameter('encoder_ppr').value
        self.wheel_circum = math.pi * self.wheel_diam

        self.wheel_names = ['front_left', 'front_right', 'rear_left', 'rear_right']
        # Store last received data: (rpm, steering_deg)
        self.wheel_data = {name: (0.0, 0.0) for name in self.wheel_names}
        self.last_time = self.get_clock().now()

        # Subscribers to each wheel's encoder feedback
        for name in self.wheel_names:
            topic = f'mobile/wheel_{name}/encoder_angle'
            self.create_subscription(Int64MultiArray, topic, 
                                     lambda msg, n=name: self.feedback_callback(msg, n), 10)

        # Odometry publisher and TF broadcaster
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Timer to compute and publish odometry (50 Hz)
        self.timer = self.create_timer(0.02, self.odometry_callback)

    def feedback_callback(self, msg, wheel_name):
        if len(msg.data) >= 2:
            self.wheel_data[wheel_name] = (float(msg.data[0]), float(msg.data[1]))

    def wheel_velocities_to_robot_twist(self):
        """
        Approximate robot velocity from average of four wheels.
        For 4WS, the instantaneous velocity of the robot center can be derived
        from any wheel if steering angles are known. Here we average all wheels.
        """
        vx_sum = 0.0
        vz_sum = 0.0
        n = 0
        for name, (rpm, steer_deg) in self.wheel_data.items():
            if rpm == 0.0:
                continue
            # Wheel linear velocity (m/s)
            wheel_speed = (rpm / 60.0) * self.wheel_circum
            # Steering angle in radians
            steer_rad = math.radians(steer_deg)
            # Contribution to robot's longitudinal and angular velocity
            # v_robot = wheel_speed * cos(steer_angle)
            # For angular velocity, using geometry: ω = (v_wheel * sin(steer_angle)) / lateral_offset
            # Simplified: take average of all wheels
            vx_sum += wheel_speed * math.cos(steer_rad)
            # Angular component: + for left, - for right (sign adjusted)
            sign = 1.0 if 'right' in name else -1.0  # left wheels produce positive yaw?
            vz_sum += wheel_speed * math.sin(steer_rad) * sign / (self.wheel_sep / 2.0)
            n += 1

        if n == 0:
            return 0.0, 0.0
        vx = vx_sum / n
        vz = vz_sum / n
        # Clamp unrealistic values
        return vx, vz

    def odometry_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 0.1:
            self.last_time = now
            return

        vx, vz = self.wheel_velocities_to_robot_twist()

        # Integrate pose
        if abs(vz) < 1e-6:
            delta_x = vx * dt * math.cos(self.theta)
            delta_y = vx * dt * math.sin(self.theta)
            delta_theta = 0.0
        else:
            delta_theta = vz * dt
            delta_x = (vx / vz) * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            delta_y = (vx / vz) * (-math.cos(self.theta + delta_theta) + math.cos(self.theta))

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize angle
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vz
        self.odom_pub.publish(odom)

        # Publish TF
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = FourWSOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()