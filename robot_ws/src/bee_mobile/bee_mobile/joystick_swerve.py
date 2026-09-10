#!/usr/bin/env python3
# 
# Joystick Swerve Controller node for ROS2
#
# This node reads joystick commands, applies deadband and exponential smoothing,
# and publishes swerve-style velocity commands to /cmd_vel_joy for twist_mux.
# It also manages trajectory recording, manual TASK waypoint requests, speed
# modes, and selectable straight or zero-turn driving modes.
# 

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, NavSatFix, Imu
from std_msgs.msg import Int8, Empty
import os
import threading
import subprocess
import signal

class JoystickSwerve(Node):
    """
    Reads a joystick and publishes /cmd_vel_joy for twist_mux.
    Features: Asynchronous continuous smoothing, external process management.
    """
    def __init__(self):
        super().__init__('joystick_swerve')

        # Axis mapping (Left Joystick Only)
        self.declare_parameter('speed_axis', 1)  # Left joystick up/down.
        self.declare_parameter('steer_axis', 3)  # Right joystick left/right.
        self.declare_parameter('dpad_x_axis', 6)
        self.declare_parameter('dpad_y_axis', 7)
        self.declare_parameter('deadband', 0.1)

        # Speed profile values for the three joystick speed modes
        self.declare_parameter(
            'speed_profiles',
            [
                0.15, 0.20,  # Slow mode: vx, wz
                0.45, 0.45,  # Normal mode: vx, wz
                0.70, 0.70,  # Fast mode: vx, wz
            ]
        )

        # Explicit button mapping
        self.declare_parameter('btn_straight', 0)        # A
        self.declare_parameter('btn_zeroturn', 1)        # B
        self.declare_parameter('btn_deadman', 5)         # R2/RT axis (Hardware trigger)
        self.declare_parameter('btn_start_record', 9)    # L3 (left joystick click).
        self.declare_parameter('btn_stop_record', 10)    # R3 (right joystick click).

        # Fetch parameters
        self.speed_axis = self.get_parameter('speed_axis').value
        self.steer_axis = self.get_parameter('steer_axis').value
        self.dpad_x_axis = self.get_parameter('dpad_x_axis').value
        self.dpad_y_axis = self.get_parameter('dpad_y_axis').value
        self.deadband = self.get_parameter('deadband').value
        
        speed_values = self.get_parameter('speed_profiles').value
        if len(speed_values) != 6:
            raise RuntimeError('speed_profiles parameter must contain exactly 6 float values')
        
        self.speed_profiles = [
            speed_values[0:2],
            speed_values[2:4],
            speed_values[4:6],
        ]
        self.speed_mode = 1

        self.btn_straight = self.get_parameter('btn_straight').value
        self.btn_zeroturn = self.get_parameter('btn_zeroturn').value
        self.btn_start_record = self.get_parameter('btn_start_record').value
        self.btn_stop_record = self.get_parameter('btn_stop_record').value
        self.btn_deadman = self.get_parameter('btn_deadman').value

        # Publisher for the manual recording signal.
        self.task_wp_pub = self.create_publisher(Empty, '/save_task_waypoint', 10)
        
        # Exponential Smoothing Filter Coefficients
        self.declare_parameter('alpha_filter', 0.10)
        self.declare_parameter('alpha_filter_wz', 0.05)
        self.alpha = self.get_parameter('alpha_filter').value
        self.alpha_wz = self.get_parameter('alpha_filter_wz').value

        self.filtered_joy_x = 0.0
        self.filtered_joy_yaw = 0.0

        # Subprocess Management & Input State
        self.recorder_process = None
        self.last_record_btns = [0, 0]
        self.last_toggle_time = 0.0
        self.last_dpad_x = 0.0
        self.last_dpad_y = 0.0
        self.last_buttons = []
        
        # State Initialization
        self.current_mode = 1 # Force Straight Mode by default
        self.target_joy_x = 0.0
        self.target_joy_yaw = 0.0
        self.deadman_active = False

        # Thread-safe GPS storage
        self.gps_lock = threading.Lock()
        self.latest_fix = None
        self.latest_imu = None

        # Fixed frequency control loop (20Hz) guarantees deceleration curve
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Publishers / Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.mode_pub = self.create_publisher(Int8, '/joystick_control_mode', 10)
        
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.fix_sub = self.create_subscription(NavSatFix, '/fix', self.fix_callback, 10)
        self.heading_sub = self.create_subscription(Imu, '/heading_imu', self.imu_callback, 10)

    def fix_callback(self, msg):
        with self.gps_lock:
            self.latest_fix = msg
    
    def imu_callback(self, msg):
        with self.gps_lock:
            self.latest_imu = msg

    def joy_callback(self, msg):
        """Hardware interrupt logic. Extracts data and updates state variables."""
        joy_x = msg.axes[self.speed_axis]
        joy_yaw = msg.axes[self.steer_axis]
        
        dpad_x = msg.axes[self.dpad_x_axis] if len(msg.axes) > self.dpad_x_axis else 0.0
        dpad_y = msg.axes[self.dpad_y_axis] if len(msg.axes) > self.dpad_y_axis else 0.0

        # Mechanical sensor noise rejection
        if abs(joy_x) < self.deadband: joy_x = 0.0
        if abs(joy_yaw) < self.deadband: joy_yaw = 0.0

        # --- L1/R1 LOGIC: TRAJECTORY RECORDING (SUBPROCESS MANAGEMENT) ---
        current_time = time.time()
        
        # Two-second software lockout.
        if current_time - self.last_toggle_time < 2.0:
            btn_l1 = 0
            btn_r1 = 0
        else:
            btn_l1 = msg.buttons[self.btn_start_record] if len(msg.buttons) > self.btn_start_record else 0
            btn_r1 = msg.buttons[self.btn_stop_record] if len(msg.buttons) > self.btn_stop_record else 0

        # Start Recording
        if btn_l1 == 1 and self.last_record_btns[0] == 0:
            if self.recorder_process is None:
                self.get_logger().info("Starting trajectory recorder...")
                try:
                    self.recorder_process = subprocess.Popen(
                        ['ros2', 'run', 'bee_mobile', 'trajectory_recorder'],
                        preexec_fn=os.setsid
                    )
                    self.last_toggle_time = current_time 
                except Exception as e:
                    self.get_logger().error(f"Fatal error at launch : {e}")
            else:
                self.get_logger().warn("Recorder is already running.")

        # Stop Recording
        if btn_r1 == 1 and self.last_record_btns[1] == 0:
            if self.recorder_process is not None:
                self.get_logger().info("Stopping trajectory recorder...")
                try:
                    os.killpg(os.getpgid(self.recorder_process.pid), signal.SIGINT)
                    self.recorder_process.wait(timeout=2.0)
                    self.last_toggle_time = current_time
                except subprocess.TimeoutExpired:
                    self.get_logger().warn("The node is not responding; forcing termination (SIGKILL).")
                    os.killpg(os.getpgid(self.recorder_process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass 
                finally:
                    self.recorder_process = None
            else:
                self.get_logger().warn("No recording to stop.")

        self.last_record_btns = [btn_l1, btn_r1]

        # --- D-PAD LOGIC: SPEED MODES & MANUAL GPS ---
        if dpad_y == 1.0 and self.last_dpad_y != 1.0:
            self.speed_mode = 0
            self.get_logger().info("Speed Mode 1: Slow")
        elif dpad_x == -1.0 and self.last_dpad_x != -1.0:
            self.speed_mode = 1
            self.get_logger().info("Speed Mode 2: Normal")
        elif dpad_y == -1.0 and self.last_dpad_y != -1.0:
            self.speed_mode = 2
            self.get_logger().info("Speed Mode 3: Fast")
            
        if dpad_x == 1.0 and self.last_dpad_x != 1.0:
            if self.recorder_process is not None:
                self.task_wp_pub.publish(Empty())
                self.get_logger().info("WP_TASK signal sent to the recorder.")
            else:
                self.get_logger().warn("Unable to send waypoint: no recording is active.")

        self.last_dpad_x = dpad_x
        self.last_dpad_y = dpad_y

        # --- MODE SELECTION ---
        buttons = msg.buttons
        if not self.last_buttons:
            self.last_buttons = [0] * len(buttons)

        max_btn_index = max(self.btn_straight, self.btn_zeroturn)
        if len(buttons) > max_btn_index:
            if buttons[self.btn_straight] and not self.last_buttons[self.btn_straight]:
                if self.current_mode != 1:
                    self.current_mode = 1
                    self.get_logger().info("Mode: Straight / Opposite")
            elif buttons[self.btn_zeroturn] and not self.last_buttons[self.btn_zeroturn]:
                if self.current_mode != 3:
                    self.current_mode = 3
                    self.get_logger().info("Mode: Zero Turn")

        self.last_buttons = list(buttons)

        # --- TARGET ASSIGNMENT ---
        self.target_joy_x = joy_x
        self.target_joy_yaw = joy_yaw

        self.deadman_active = (msg.axes[self.btn_deadman] < 0.0)
        
        if self.deadman_active:
            mode_msg = Int8()
            mode_msg.data = self.current_mode
            self.mode_pub.publish(mode_msg)

    def control_loop(self):
        """
        Runs asynchronously at 20Hz.
        """
        if not self.deadman_active:
            self.target_joy_x = 0.0
            self.target_joy_yaw = 0.0

        # Exponential Smoothing (Low-Pass Filter)
        self.filtered_joy_x = (self.alpha * self.target_joy_x) + ((1.0 - self.alpha) * self.filtered_joy_x)
        self.filtered_joy_yaw = (self.alpha_wz * self.target_joy_yaw) + ((1.0 - self.alpha_wz) * self.filtered_joy_yaw)

        # Truncation limits hardware whining noise from motors receiving micro-voltages near 0
        if abs(self.filtered_joy_x) < 0.001: self.filtered_joy_x = 0.0
        if abs(self.filtered_joy_yaw) < 0.001: self.filtered_joy_yaw = 0.0

        # MUX LOCKOUT PREVENTION
        if not self.deadman_active and self.filtered_joy_x == 0.0 and self.filtered_joy_yaw == 0.0:
            return

        current_max_lin = self.speed_profiles[self.speed_mode][0]
        current_max_ang = self.speed_profiles[self.speed_mode][1]

        twist = Twist()

        # Kinematic Mode Enforcement
        if self.current_mode == 1: # Straight / Opposite
            twist.linear.x = self.filtered_joy_x * current_max_lin
            twist.angular.z = self.filtered_joy_yaw * current_max_ang
        elif self.current_mode == 3: # Zero-Turn
            twist.angular.z = self.filtered_joy_yaw * current_max_ang
    
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickSwerve()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.recorder_process is not None:
            node.recorder_process.send_signal(signal.SIGINT)
            node.recorder_process.wait()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()