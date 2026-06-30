#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class WheelStateConverter(Node):
    """
    Subscribes to the four wheel encoder topics (/mobile/wheel_*/encoder_angle, Float64MultiArray) and 
    republishes as a unified JointState (radians) on /wheel_joint_states for RViz.
    data[0] = wheel rolling velocity (RPM) → integrated to position
    data[1] = steering angle (degrees) → converted to radians
    """
    def __init__(self):
        super().__init__('wheel_state_converter')

        # Parameters
        # Publish wheel joint states at a higher default rate to keep RViz
        # visuals synchronized with odometry/tf (match odom 50Hz by default).
        self.declare_parameter('publish_rate', 50.0)
        self.publish_rate = self.get_parameter('publish_rate').value

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/wheel_joint_states', 10)

        # Subscribers
        self.create_subscription(Float64MultiArray, '/mobile/wheel_front_left/encoder_angle',  self.fl_callback, 10)
        self.create_subscription(Float64MultiArray, '/mobile/wheel_front_right/encoder_angle', self.fr_callback, 10)
        self.create_subscription(Float64MultiArray, '/mobile/wheel_rear_left/encoder_angle',   self.rl_callback, 10)
        self.create_subscription(Float64MultiArray, '/mobile/wheel_rear_right/encoder_angle',  self.rr_callback, 10)

        # Wheel rolling velocities (RPM → rad/s)
        self.joint_velocities = {
            'front_left_wheel_joint':  0.0,
            'front_right_wheel_joint': 0.0,
            'rear_left_wheel_joint':   0.0,
            'rear_right_wheel_joint':  0.0,
        }

        # All joint positions (steering in rad, wheel integrated from velocity)
        self.joint_positions = {
            'front_left_wheel_joint':    0.0,
            'front_left_steering_joint': 0.0,
            'front_right_wheel_joint':   0.0,
            'front_right_steering_joint':0.0,
            'rear_left_wheel_joint':     0.0,
            'rear_left_steering_joint':  0.0,
            'rear_right_wheel_joint':    0.0,
            'rear_right_steering_joint': 0.0,
        }

        self.last_time = self.get_clock().now()

        # Publish timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_wheel_joint_states)

        self.get_logger().info("Wheel state converter node started - publishes /wheel_joint_states (8 joints) in radians")

    def process_encoder_msg(self, msg, wheel_prefix):
        if len(msg.data) >= 2:
            # data[0]: rolling velocity in RPM → rad/s
            self.joint_velocities[f'{wheel_prefix}_wheel_joint'] = float(msg.data[0]) * 2 * math.pi / 60.0

            # data[1]: steering angle in degrees → radians
            self.joint_positions[f'{wheel_prefix}_steering_joint'] = float(msg.data[1]) * math.pi / 180.0
        else:
            self.get_logger().warn(f"Malformed message on {wheel_prefix} (expected 2 elements)", once=True)

    def fl_callback(self, msg): self.process_encoder_msg(msg, 'front_left')
    def fr_callback(self, msg): self.process_encoder_msg(msg, 'front_right')
    def rl_callback(self, msg): self.process_encoder_msg(msg, 'rear_left')
    def rr_callback(self, msg): self.process_encoder_msg(msg, 'rear_right')

    def publish_wheel_joint_states(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Integrate velocity → position for wheel joints
        for name, vel in self.joint_velocities.items():
            self.joint_positions[name] += vel * dt

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name     = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        msg.velocity = [self.joint_velocities.get(n, 0.0) for n in msg.name]
        msg.effort   = [0.0] * len(msg.name)  # Initialize efforts to 0.0

        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelStateConverter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print(f"\n[INFO] [{node.get_name()}]: Shutdown requested by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()