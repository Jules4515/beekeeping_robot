#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from sensor_msgs.msg import Joy
import math

class FourWSJoystickControl(Node):
    def __init__(self):
        super().__init__('four_ws_joystick_control')

        # ---------- Parameters ----------
        self.declare_parameter('wheel_diameter_m', 0.43)
        self.declare_parameter('max_speed_kmh', 10.0)
        self.declare_parameter('max_steering_deg', 120.0)
        self.declare_parameter('wheel_base_m', 0.96)
        self.declare_parameter('wheel_separation_m', 0.83)
        self.declare_parameter('speed_axis', 1)
        self.declare_parameter('steer_axis', 0)
        self.declare_parameter('rotate_axis', 3)
        self.declare_parameter('deadband', 0.5)
        self.declare_parameter('button_opposite', 2)
        self.declare_parameter('button_crab', 3)
        self.declare_parameter('button_zeroturn', 1)
        self.declare_parameter('button_straight', 0)
        self.declare_parameter('zero_steering_min_deg', 1.0)   # ESP32 deadzone workaround
        self.declare_parameter('cmd_deadband_deg', 0.0)        # new: stop sending identical commands

        # Get parameters
        self.wheel_diameter = self.get_parameter('wheel_diameter_m').value
        self.max_speed_kmh = self.get_parameter('max_speed_kmh').value
        self.max_steering_deg = self.get_parameter('max_steering_deg').value
        self.wheel_base = self.get_parameter('wheel_base_m').value
        self.wheel_sep = self.get_parameter('wheel_separation_m').value
        self.speed_axis = self.get_parameter('speed_axis').value
        self.steer_axis = self.get_parameter('steer_axis').value
        self.rotate_axis = self.get_parameter('rotate_axis').value
        self.deadband = self.get_parameter('deadband').value
        self.btn_opposite = self.get_parameter('button_opposite').value
        self.btn_crab = self.get_parameter('button_crab').value
        self.btn_zeroturn = self.get_parameter('button_zeroturn').value
        self.btn_straight = self.get_parameter('button_straight').value
        self.zero_min_deg = self.get_parameter('zero_steering_min_deg').value
        self.cmd_deadband = self.get_parameter('cmd_deadband_deg').value

        # Wheel modules
        self.wheel_modules = ['front_left', 'front_right', 'rear_left', 'rear_right']
        self.wheel_pubs = {}
        for name in self.wheel_modules:
            topic = f'mobile/wheel_{name}/motor_speed'
            self.wheel_pubs[name] = self.create_publisher(Int64MultiArray, topic, 10)

        # Feedback subscribers
        self.feedback_subs = {}
        self.actual_angle = {name: 0.0 for name in self.wheel_modules}
        for name in self.wheel_modules:
            topic = f'mobile/wheel_{name}/encoder_angle'
            self.feedback_subs[name] = self.create_subscription(
                Int64MultiArray, topic,
                lambda msg, n=name: self.feedback_callback(msg, n),
                10
            )

        # Mode management
        self.mode = 0
        self.mode_names = {
            0: 'No mode',
            1: 'Opposite (4WS)',
            2: 'Crab (Parallel)',
            3: 'Zero Turn (Pivot)',
            4: 'Steering Zero (Straight)'
        }

        # Button states
        self.last_btn_opposite = False
        self.last_btn_crab = False
        self.last_btn_zeroturn = False
        self.last_btn_straight = False

        # Last commanded values (for deadband)
        self.last_steer_cmd = {name: None for name in self.wheel_modules}
        self.last_rpm_cmd = {name: None for name in self.wheel_modules}

        # Subscriptions
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.display_timer = self.create_timer(0.5, self.display_status)

        self.get_logger().info("4WS Node Ready – A button = Steering Zero ({}°), cmd deadband = {}°".format(
            self.zero_min_deg, self.cmd_deadband))

    def feedback_callback(self, msg, wheel_name):
        if len(msg.data) >= 2:
            self.actual_angle[wheel_name] = float(msg.data[1])

    def rpm_from_speed_kmh(self, speed_kmh):
        circumference_m = math.pi * self.wheel_diameter
        rpm = (speed_kmh * 1000.0) / (circumference_m * 60.0)
        return int(rpm)

    def compute_steering_angles(self, vx_kmh, steering_cmd_deg, rotate_cmd):
        angles = {'front_left': 0.0, 'front_right': 0.0,
                  'rear_left': 0.0, 'rear_right': 0.0}

        if self.mode == 1:          # Opposite
            base = steering_cmd_deg
            angles['front_left'] = base
            angles['front_right'] = -base
            angles['rear_left'] = -base
            angles['rear_right'] = base
        elif self.mode == 2:        # Crab
            base = steering_cmd_deg
            angles = {k: base for k in angles}
        elif self.mode == 3:        # Zero turn
            track = self.wheel_sep
            if track > 0:
                pivot = math.degrees(math.atan2(self.wheel_base, track))
                angles['front_left'] = -pivot
                angles['front_right'] = pivot
                angles['rear_left'] = pivot
                angles['rear_right'] = -pivot
            else:
                angles = {k: 45.0 if 'right' in k else -45.0 for k in angles}
        elif self.mode == 4:        # Steering Zero – use minimum angle (workaround)
            angles = {k: self.zero_min_deg for k in angles}
        # else mode 0: all zero already

        # Clamp
        for k in angles:
            angles[k] = max(-self.max_steering_deg, min(self.max_steering_deg, angles[k]))
        return angles

    def should_publish(self, wheel_name, new_angle, new_rpm):
        """Return True if command has changed beyond deadband."""
        last_angle = self.last_steer_cmd[wheel_name]
        last_rpm = self.last_rpm_cmd[wheel_name]
        if last_angle is None or last_rpm is None:
            return True
        if abs(new_angle - last_angle) > self.cmd_deadband:
            return True
        if new_rpm != last_rpm:
            return True
        # Also publish if the robot just became stationary (send a final zero command)
        if new_rpm == 0 and last_rpm != 0:
            return True
        return False

    def joy_callback(self, msg):
        # Axes
        speed_val = msg.axes[self.speed_axis] if len(msg.axes) > self.speed_axis else 0.0
        steer_val = msg.axes[self.steer_axis] if len(msg.axes) > self.steer_axis else 0.0
        rotate_val = msg.axes[self.rotate_axis] if len(msg.axes) > self.rotate_axis else 0.0

        if abs(speed_val) < self.deadband: speed_val = 0.0
        if abs(steer_val) < self.deadband: steer_val = 0.0
        if abs(rotate_val) < self.deadband: rotate_val = 0.0

        # Buttons
        btn_opp = (len(msg.buttons) > self.btn_opposite and msg.buttons[self.btn_opposite] == 1)
        btn_crab = (len(msg.buttons) > self.btn_crab and msg.buttons[self.btn_crab] == 1)
        btn_zero = (len(msg.buttons) > self.btn_zeroturn and msg.buttons[self.btn_zeroturn] == 1)
        btn_straight = (len(msg.buttons) > self.btn_straight and msg.buttons[self.btn_straight] == 1)

        # Edge detection
        if btn_opp and not self.last_btn_opposite:
            self.mode = 1
            self.get_logger().info(f"Mode -> {self.mode_names[1]}")
        if btn_crab and not self.last_btn_crab:
            self.mode = 2
            self.get_logger().info(f"Mode -> {self.mode_names[2]}")
        if btn_zero and not self.last_btn_zeroturn:
            self.mode = 3
            self.get_logger().info(f"Mode -> {self.mode_names[3]}")
        if btn_straight and not self.last_btn_straight:
            self.mode = 4
            self.get_logger().info(f"Mode -> {self.mode_names[4]} (sending {self.zero_min_deg}°)")

        self.last_btn_opposite = btn_opp
        self.last_btn_crab = btn_crab
        self.last_btn_zeroturn = btn_zero
        self.last_btn_straight = btn_straight

        # If no mode, stop and clear last commands
        if self.mode == 0:
            for name in self.wheel_modules:
                if self.should_publish(name, 0, 0):
                    self.wheel_pubs[name].publish(Int64MultiArray(data=[0, 0]))
                    self.last_steer_cmd[name] = 0
                    self.last_rpm_cmd[name] = 0
            return

        # Speed and angles
        target_speed_kmh = speed_val * self.max_speed_kmh
        base_steering_deg = steer_val * self.max_steering_deg
        wheel_angles = self.compute_steering_angles(target_speed_kmh, base_steering_deg, rotate_val)

        # RPMs
        if self.mode == 3:  # Zero turn uses rotate axis
            max_rpm = self.rpm_from_speed_kmh(self.max_speed_kmh)
            rpm_left = -int(rotate_val * max_rpm)
            rpm_right = int(rotate_val * max_rpm)
            rpm = {'front_left': rpm_left, 'front_right': rpm_right,
                   'rear_left': rpm_left, 'rear_right': rpm_right}
        else:
            rpm_val = self.rpm_from_speed_kmh(target_speed_kmh)
            rpm = {k: rpm_val for k in self.wheel_modules}

        # Publish only on change
        for name in self.wheel_modules:
            new_angle = wheel_angles[name]
            new_rpm = rpm[name]
            if self.should_publish(name, new_angle, new_rpm):
                cmd = Int64MultiArray(data=[new_rpm, int(new_angle)])
                self.wheel_pubs[name].publish(cmd)
                self.last_steer_cmd[name] = new_angle
                self.last_rpm_cmd[name] = new_rpm

    def display_status(self):
        self.get_logger().info(
            f"Mode: {self.mode_names[self.mode]} | "
            f"FB: FL={self.actual_angle['front_left']:.1f}° FR={self.actual_angle['front_right']:.1f}° "
            f"RL={self.actual_angle['rear_left']:.1f}° RR={self.actual_angle['rear_right']:.1f}°"
        )

def main(args=None):
    rclpy.init(args=args)
    node = FourWSJoystickControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()