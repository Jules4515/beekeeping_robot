#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

class JoystickModeNode(Node):
    """
    Reads a joystick (Xbox 360) and publishes:
      - /cmd_vel (Twist)   → linear.x, linear.y, angular.z
      - /mode_select (Int32) → 1=Opposite, 2=Crab, 3=Zero turn, 4=Straight
    """
    def __init__(self):
        super().__init__('joystick_mode_node')

        # Parameters
        self.declare_parameter('speed_axis', 1)          # left stick vertical
        self.declare_parameter('steer_axis', 0)          # left stick horizontal
        self.declare_parameter('rotate_axis', 3)         # right stick horizontal
        self.declare_parameter('deadband', 0.1)
        self.declare_parameter('max_linear', 7.5)        # m/s
        self.declare_parameter('max_angular', 10.0)      # rad/s

        # Button indices (Xbox 360: A=0, B=1, X=2, Y=3)
        self.declare_parameter('btn_opposite', 2)        # X
        self.declare_parameter('btn_crab', 3)            # Y
        self.declare_parameter('btn_zeroturn', 1)        # B
        self.declare_parameter('btn_straight', 0)        # A

        self.speed_axis = self.get_parameter('speed_axis').value
        self.steer_axis = self.get_parameter('steer_axis').value
        self.rotate_axis = self.get_parameter('rotate_axis').value
        self.deadband = self.get_parameter('deadband').value
        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value
        self.btn_opp = self.get_parameter('btn_opposite').value
        self.btn_crab = self.get_parameter('btn_crab').value
        self.btn_zero = self.get_parameter('btn_zeroturn').value
        self.btn_str = self.get_parameter('btn_straight').value

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(Int32, '/mode_select', 10)

        # Subscriber
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Mode state
        self.current_mode = 0
        self.last_btn_state = {self.btn_opp: False, self.btn_crab: False,
                               self.btn_zero: False, self.btn_str: False}

        self.get_logger().info("Joystick node started – publishes /cmd_vel and /mode_select")

    def joy_callback(self, msg):
        # Read axes with deadband
        lin_x = msg.axes[self.speed_axis] if len(msg.axes) > self.speed_axis else 0.0
        lin_y = msg.axes[self.steer_axis] if len(msg.axes) > self.steer_axis else 0.0
        ang_z = msg.axes[self.rotate_axis] if len(msg.axes) > self.rotate_axis else 0.0

        if abs(lin_x) < self.deadband: lin_x = 0.0
        if abs(lin_y) < self.deadband: lin_y = 0.0
        if abs(ang_z) < self.deadband: ang_z = 0.0

        # Scale to real units
        twist = Twist()
        twist.linear.x = lin_x * self.max_lin
        twist.linear.y = lin_y * self.max_lin
        twist.angular.z = ang_z * self.max_ang
        self.cmd_vel_pub.publish(twist)

        # Mode selection (rising edge detection)
        buttons = msg.buttons if len(msg.buttons) > max(self.btn_opp, self.btn_crab, self.btn_zero, self.btn_str) else []
        new_mode = self.current_mode

        if buttons and buttons[self.btn_opp] and not self.last_btn_state[self.btn_opp]:
            new_mode = 1
            self.get_logger().info("Mode: Opposite (4WS)")
        elif buttons and buttons[self.btn_crab] and not self.last_btn_state[self.btn_crab]:
            new_mode = 2
            self.get_logger().info("Mode: Crab (Parallel)")
        elif buttons and buttons[self.btn_zero] and not self.last_btn_state[self.btn_zero]:
            new_mode = 3
            self.get_logger().info("Mode: Zero Turn (Pivot)")
        elif buttons and buttons[self.btn_str] and not self.last_btn_state[self.btn_str]:
            new_mode = 4
            self.get_logger().info("Mode: Steering Zero (Straight)")

        # Update button states
        if buttons:
            self.last_btn_state[self.btn_opp] = buttons[self.btn_opp]
            self.last_btn_state[self.btn_crab] = buttons[self.btn_crab]
            self.last_btn_state[self.btn_zero] = buttons[self.btn_zero]
            self.last_btn_state[self.btn_str] = buttons[self.btn_str]

        # Publish mode only if changed
        if new_mode != self.current_mode:
            self.current_mode = new_mode
            mode_msg = Int32()
            mode_msg.data = self.current_mode
            self.mode_pub.publish(mode_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickModeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()