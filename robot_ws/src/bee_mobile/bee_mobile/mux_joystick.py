#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8

class MuxJoystick(Node):
    """
    Reads a joystick and publishes /cmd_vel_joy for twist_mux.
    Uses dedicated buttons (A, B, X, Y) to lock specific movement axes.
    """
    def __init__(self):
        super().__init__('mux_joystick')

        # Axis mapping
        self.declare_parameter('speed_axis', 1)
        self.declare_parameter('steer_axis', 0)
        self.declare_parameter('rotate_axis', 3)
        self.declare_parameter('deadband', 0.1)

        # Speed limits
        self.declare_parameter('max_linear_speed_ms', 0.70)#0.07
        self.declare_parameter('max_angular_speed_rads', 0.50)#0.09

        # Explicit button mapping (Xbox 360/One standard)
        self.declare_parameter('btn_straight', 0)        # A button
        self.declare_parameter('btn_zeroturn', 1)        # B button
        self.declare_parameter('btn_holonome', 2)        # X button
        self.declare_parameter('btn_crab', 3)            # Y button
        self.declare_parameter('btn_deadman', 5)         # R2 button

        # Fetch parameters
        self.speed_axis = self.get_parameter('speed_axis').value
        self.steer_axis = self.get_parameter('steer_axis').value
        self.rotate_axis = self.get_parameter('rotate_axis').value
        self.deadband = self.get_parameter('deadband').value
        
        self.max_lin = self.get_parameter('max_linear_speed_ms').value
        self.max_ang = self.get_parameter('max_angular_speed_rads').value

        self.btn_straight = self.get_parameter('btn_straight').value
        self.btn_zeroturn = self.get_parameter('btn_zeroturn').value
        self.btn_holonome = self.get_parameter('btn_holonome').value
        self.btn_crab = self.get_parameter('btn_crab').value
        self.btn_deadman = self.get_parameter('btn_deadman').value

        # ---------------------------------------------------------
        # Exponential Smoothing Filter State
        # WHY: Maintains the previous cycle's filtered values to compute 
        # the asymptotic ramp, preventing raw joystick steps from shocking the mechanics.
        # ---------------------------------------------------------
        self.declare_parameter('alpha_filter', 0.10)
        self.declare_parameter('alpha_filter_wz', 0.05)
        self.alpha = self.get_parameter('alpha_filter').value
        self.alpha_wz = self.get_parameter('alpha_filter_wz').value

        self.filtered_joy_x = 0.0
        self.filtered_joy_y = 0.0
        self.filtered_joy_z = 0.0

        # State tracking for the deadman switch
        self.was_deadman_pressed = False

        # Publisher for twist_mux
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Publisher for mode and deadman state propagation
        self.mode_pub = self.create_publisher(Int8, '/joystick_control_mode', 10)

        # Initial state (0 = Holonome/Free)
        self.current_mode = 0 
        self.last_buttons = []

        self.get_logger().info("Mux Joystick Node started - Publishes to /cmd_vel_joy")
        self.get_logger().info("Modes: X=Holonome, Y=Crab, B=ZeroTurn, A=Straight")

    def joy_callback(self, msg):
        # Read raw axes
        joy_x = msg.axes[self.speed_axis]
        joy_y = msg.axes[self.steer_axis]
        joy_z = msg.axes[self.rotate_axis]

        # Apply deadband
        if abs(joy_x) < self.deadband: joy_x = 0.0
        if abs(joy_y) < self.deadband: joy_y = 0.0
        if abs(joy_z) < self.deadband: joy_z = 0.0

        # ---------------------------------------------------------
        # Low-Pass Filter Math
        # WHY: Smooths the 0-to-1 step input into a continuous curve.
        # A smaller alpha makes the robot feel heavier and smoother.
        # ---------------------------------------------------------
        self.filtered_joy_x = (self.alpha * joy_x) + ((1.0 - self.alpha) * self.filtered_joy_x)
        self.filtered_joy_y = (self.alpha * joy_y) + ((1.0 - self.alpha) * self.filtered_joy_y)
        self.filtered_joy_z = (self.alpha_wz * joy_z) + ((1.0 - self.alpha_wz) * self.filtered_joy_z)

        buttons = msg.buttons
        if not self.last_buttons:
            self.last_buttons = [0] * len(buttons)

        # Rising edge detection for explicit mode switching
        max_btn_index = max(self.btn_straight, self.btn_crab, self.btn_zeroturn, self.btn_holonome)
        if len(buttons) > max_btn_index:
            
            if buttons[self.btn_holonome] and not self.last_buttons[self.btn_holonome]:
                self.current_mode = 0
                self.get_logger().info("Mode: Holonome (X pressed)")
                
            elif buttons[self.btn_straight] and not self.last_buttons[self.btn_straight]:
                self.current_mode = 1
                self.get_logger().info("Mode: Straight (A pressed)")
            
            elif buttons[self.btn_crab] and not self.last_buttons[self.btn_crab]:
                self.current_mode = 2
                self.get_logger().info("Mode: Crab (Y pressed)")
            
            elif buttons[self.btn_zeroturn] and not self.last_buttons[self.btn_zeroturn]:
                self.current_mode = 3
                self.get_logger().info("Mode: Zero Turn (B pressed)")

        self.last_buttons = list(buttons)

        # --- DEADMAN SWITCH LOGIC ---
        # Check if deadman button (e.g., R2) is pressed (value less than 0)
        deadman_pressed = (msg.axes[self.btn_deadman] < 0.0) 

        # Publish the hardware state to the kinematics node
        if deadman_pressed:
            # Publish the active mode continuously while deadman is held
            mode_msg = Int8()
            mode_msg.data = self.current_mode
            self.mode_pub.publish(mode_msg)
        else:
            # Silence radio: Do nothing if deadman is not pressed
            pass

        # Translate joystick input to Twist message based on active mode
        twist = Twist()

        if deadman_pressed:
            # Deadman active: Apply standard driving modes USING FILTERED INPUTS
            if self.current_mode == 1:       # Straight (Locks lateral Y movement)
                twist.linear.x = self.filtered_joy_x * self.max_lin
                twist.linear.y = 0.0
                twist.angular.z = self.filtered_joy_z * self.max_ang
                
            elif self.current_mode == 2:     # Crab (Locks Z rotation)
                twist.linear.x = self.filtered_joy_x * self.max_lin
                twist.linear.y = self.filtered_joy_y * self.max_lin
                twist.angular.z = 0.0
                
            elif self.current_mode == 3:     # Zero Turn (Locks X and Y translations)
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = self.filtered_joy_z * self.max_ang
                
            else:                            # Holonome (Mode 0 - All axes active)
                twist.linear.x = self.filtered_joy_x * self.max_lin
                twist.linear.y = self.filtered_joy_y * self.max_lin
                twist.angular.z = self.filtered_joy_z * self.max_ang
        
            # Publish the command and update state memory
            self.cmd_vel_pub.publish(twist)
            self.was_deadman_pressed = True

        else:
            # Deadman released
            if self.was_deadman_pressed:
                # Falling edge detected: Button was just released this exact cycle.
                self.cmd_vel_pub.publish(twist) # twist is already initialized to 0.0
                
                # ---------------------------------------------------------
                # Filter Reset
                # WHY: Crucial safety measure. If we don't reset the filter,
                # pressing the deadman switch later would resume from the old 
                # velocity memory instead of starting safely from zero.
                # ---------------------------------------------------------
                self.filtered_joy_x = 0.0
                self.filtered_joy_y = 0.0
                self.filtered_joy_z = 0.0
                
                self.was_deadman_pressed = False


def main(args=None):
    rclpy.init(args=args)
    node = MuxJoystick()
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