#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray

class WheelStateConverterNode(Node):
    """
    Subscribes to the four wheel encoder topics (/mobile/wheel_*/encoder_angle, Int64MultiArray, degrees) and 
    republishes the angles as a unified JointState (radians) on /wheel_joint_states for RViz.
    """
    def __init__(self):
        super().__init__('wheel_state_converter_node')

        # Parameters
        self.declare_parameter('publish_rate', 10.0) # Hz
        self.publish_rate = self.get_parameter('publish_rate').value

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/wheel_joint_states', 10)

        # Subscribers
        self.fl_sub = self.create_subscription(Int64MultiArray, '/mobile/wheel_front_left/encoder_angle', self.fl_callback, 10)
        self.fr_sub = self.create_subscription(Int64MultiArray, '/mobile/wheel_front_right/encoder_angle', self.fr_callback, 10)
        self.rl_sub = self.create_subscription(Int64MultiArray, '/mobile/wheel_rear_left/encoder_angle', self.rl_callback, 10)
        self.rr_sub = self.create_subscription(Int64MultiArray, '/mobile/wheel_rear_right/encoder_angle', self.rr_callback, 10)

        # Joint state dictionary tracking current positions (in radians)
        self.joint_positions = {
            'front_left_joint': 0.0,
            'front_left_turn_joint': 0.0,
            'front_right_joint': 0.0,
            'front_right_turn_joint': 0.0,
            'rear_left_joint': 0.0,
            'rear_left_turn_joint': 0.0,
            'rear_right_joint': 0.0,
            'rear_right_turn_joint': 0.0
        }

        # Timer for publishing at a fixed rate
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_wheel_joint_states)

        self.get_logger().info("Wheel state converter node started - publishes /wheel_joint_states (8 joints) in radians")

    def degrees_to_radians(self, degrees):
        """Converts degrees to radians."""
        return float(degrees) * (math.pi / 180.0)

    def process_encoder_msg(self, msg, wheel_prefix):
        """
        Extracts rolling (index 0) and steering (index 1) angles from the message,
        converts them to radians, and updates the dictionary.
        """
        if len(msg.data) >= 2:
            # data[0] is rolling (rotation of the wheel)
            self.joint_positions[f'{wheel_prefix}_joint'] = self.degrees_to_radians(msg.data[0])
            
            # data[1] is steering (orientation of the wheel module)
            self.joint_positions[f'{wheel_prefix}_turn_joint'] = self.degrees_to_radians(msg.data[1])
        else:
            self.get_logger().warn(f"Received malformed array on {wheel_prefix} (expected 2 elements)", once=True)

    # Callbacks
    def fl_callback(self, msg):
        self.process_encoder_msg(msg, 'front_left')

    def fr_callback(self, msg):
        self.process_encoder_msg(msg, 'front_right')

    def rl_callback(self, msg):
        self.process_encoder_msg(msg, 'rear_left')

    def rr_callback(self, msg):
        self.process_encoder_msg(msg, 'rear_right')

    def publish_wheel_joint_states(self):
        """Builds and publishes the unified JointState message."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Populate the message arrays directly from the dictionary
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        
        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WheelStateConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()