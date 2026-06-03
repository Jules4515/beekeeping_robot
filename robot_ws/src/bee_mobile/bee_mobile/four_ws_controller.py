#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32
from geometry_msgs.msg import Twist
import math

class FourWSController(Node):
    """
    Subscribes to /cmd_vel and /mode_select.
    Publishes wheel steering angles and motor speeds to /mobile/wheel_*/motor_speed.
    No PID parameter sending – uses whatever the ESP32 already has.
    """
    def __init__(self):
        super().__init__('four_ws_controller')

        # ---------- Robot geometry ----------
        self.declare_parameter('wheel_diameter_m', 0.43)
        self.declare_parameter('wheel_base_m', 0.96)
        self.declare_parameter('wheel_separation_m', 0.83)
        self.declare_parameter('max_steering_deg', 80.0)

        # ---------- Software Limits (Mode Specific) ----------
        self.declare_parameter('max_linear_speed_ms', 0.34)       # Straight and Crab mode
        self.declare_parameter('max_angular_speed_rads', 0.36)      # Tighter limit for Zero Turn mode

        # ---------- Hardware Limits (Absolute) ----------
        self.declare_parameter('limit_motor_speed_rpm', 30.0)   # Absolute motor protection (Priority 1)
                                                                # equivalent to 0.675 m/s max or 1.064 rad/s (1 rotation in 6 seconds)

        # ---------- Deadbands & Offsets ----------
        self.declare_parameter('zero_steering_min_deg', 1.0)   # workaround for ESP32 deadzone
        self.declare_parameter('cmd_deadband_deg', 0.5)        # stop sending identical commands

        self.wheel_diameter = self.get_parameter('wheel_diameter_m').value
        self.wheel_base = self.get_parameter('wheel_base_m').value
        self.wheel_sep = self.get_parameter('wheel_separation_m').value
        self.max_steering_deg = self.get_parameter('max_steering_deg').value

        self.max_linear_speed_ms = self.get_parameter('max_linear_speed_ms').value
        self.max_angular_speed_rads = self.get_parameter('max_angular_speed_rads').value
        self.limit_motor_speed_rpm = self.get_parameter('limit_motor_speed_rpm').value

        self.zero_min_deg = self.get_parameter('zero_steering_min_deg').value
        self.cmd_deadband = self.get_parameter('cmd_deadband_deg').value

        # Wheel modules
        self.wheel_modules = ['front_left', 'front_right', 'rear_left', 'rear_right']
        self.wheel_pubs = {}
        for name in self.wheel_modules:
            topic = f'mobile/wheel_{name}/motor_speed'
            self.wheel_pubs[name] = self.create_publisher(Float64MultiArray, topic, 10)

        # Feedback subscribers (optional, for display only)
        self.actual_angle = {name: 0.0 for name in self.wheel_modules}
        for name in self.wheel_modules:
            topic_fb = f'mobile/wheel_{name}/encoder_angle'
            self.create_subscription(Float64MultiArray, topic_fb,
                                     lambda msg, n=name: self.feedback_callback(msg, n), 10)

        # State
        self.current_mode = 0          # 0 = none, 1=opposite, 2=crab, 3=zero, 4=straight
        self.cmd_vel = Twist()
        self.last_steer_cmd = {name: None for name in self.wheel_modules}
        self.last_rpm_cmd = {name: None for name in self.wheel_modules}

        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.mode_sub = self.create_subscription(Int32, '/mode_select', self.mode_callback, 10)

        # Control timer (50 Hz)
        self.timer = self.create_timer(0.02, self.control_callback)

        # Display timer
        self.display_timer = self.create_timer(0.5, self.display_status)

        self.get_logger().info("4WS Controller started (no PID modification)")

    def feedback_callback(self, msg, wheel_name):
        if len(msg.data) >= 2:
            self.actual_angle[wheel_name] = float(msg.data[1])

    def rpm_from_speed_ms(self, speed_ms):
        circumference_m = math.pi * self.wheel_diameter
        rpm = (speed_ms * 60.0) / circumference_m
        return int(rpm)

    def vel_callback(self, msg):
        self.cmd_vel = msg

    def mode_callback(self, msg):
        new_mode = msg.data
        if new_mode == self.current_mode:
            return
        self.current_mode = new_mode
        self.get_logger().info(f"Mode changed to: {self.mode_name(new_mode)}")

    def mode_name(self, mode):
        return {1: 'Opposite', 2: 'Crab', 3: 'Zero Turn', 4: 'Straight'}.get(mode, 'Unknown')

    def compute_steering_angles(self, steering_cmd_deg):
        angles = {name: 0.0 for name in self.wheel_modules}

        if self.current_mode == 1:      # Opposite
            base = steering_cmd_deg
            angles['front_left'] = base
            angles['front_right'] = base
            angles['rear_left'] = -base
            angles['rear_right'] = -base

        elif self.current_mode == 2:    # Crab
            base = steering_cmd_deg
            for k in angles:
                angles[k] = base

        elif self.current_mode == 3:    # Zero turn
            track = self.wheel_sep
            if track > 0:
                pivot = math.degrees(math.atan2(self.wheel_base, track))
                angles['front_left'] = -pivot
                angles['front_right'] = pivot
                angles['rear_left'] = pivot
                angles['rear_right'] = -pivot
            else:
                angles = {k: 45.0 if 'right' in k else -45.0 for k in angles}

        elif self.current_mode == 4:    # Straight mode
            offset = self.zero_min_deg
            for k in angles:
                angles[k] = offset
        # else mode 0 or unknown: stay zero
        # Clamp
        for k in angles:
            angles[k] = max(-self.max_steering_deg, min(self.max_steering_deg, angles[k]))
        return angles

    def should_publish(self, wheel_name, new_angle, new_rpm):
        """Checks if the new command is significantly different from the last sent command."""
        last_angle = self.last_steer_cmd[wheel_name]
        last_rpm = self.last_rpm_cmd[wheel_name]
        if last_angle is None or last_rpm is None:
            return True
        if abs(new_angle - last_angle) > self.cmd_deadband:
            return True
        if new_rpm != last_rpm:
            return True
        if new_rpm == 0 and last_rpm != 0:
            return True
        return False

    def control_callback(self):
        if self.current_mode == 0:
            # No mode selected: stop all wheels (send once)
            for name in self.wheel_modules:
                if self.should_publish(name, 0, 0):
                    self.wheel_pubs[name].publish(Float64MultiArray(data=[0.0, 0.0]))
                    self.last_steer_cmd[name] = 0
                    self.last_rpm_cmd[name] = 0
            return

        # Extract commands
        cmd_vx = self.cmd_vel.linear.x  # Command speed in m/s
        cmd_vy = self.cmd_vel.linear.y  # Pure steering ratio [-1.0 to 1.0]
        cmd_wz = self.cmd_vel.angular.z # Command angular speed in rad/s

        # Map the [-1.0, 1.0] joystick ratio directly to the maximum physical steering angle
        steering_cmd_deg = -cmd_vy * self.max_steering_deg
        wheel_angles = self.compute_steering_angles(steering_cmd_deg)

        # Compute RPMs for each wheel
        if self.current_mode == 3:   # Zero turn uses angular.z
            # Calculate the linear velocity of the wheel relative to the robot's center (m/s)
            r_pivot = math.sqrt((self.wheel_base/2)**2 + (self.wheel_sep/2)**2)
            cmd_wz = max(-self.max_angular_speed_rads, min(self.max_angular_speed_rads, cmd_wz)) # Angular limit (rotation of the robot)
            v_wheel_ms = cmd_wz * r_pivot
            
            rpm_val = self.rpm_from_speed_ms(abs(v_wheel_ms))
            sign = 1 if cmd_wz >= 0 else -1
            
            rpm_left = -sign * rpm_val
            rpm_right = sign * rpm_val
            rpm = {name: rpm_left if 'left' in name else rpm_right for name in self.wheel_modules}
            
        else:                        # Opposite, Crab, Straight use linear speed
            cmd_vx = max(-self.max_linear_speed_ms, min(self.max_linear_speed_ms, cmd_vx)) # Linear limit
            rpm_val = self.rpm_from_speed_ms(abs(cmd_vx))
            sign = 1 if cmd_vx >= 0 else -1
            rpm_val = sign * rpm_val
            rpm = {name: rpm_val for name in self.wheel_modules}

        # RPM limit for all 4 wheels
        rpm = {name: max(-self.limit_motor_speed_rpm, min(self.limit_motor_speed_rpm, value)) for name, value in rpm.items()}
        wheel_angles = {name: max(-self.max_steering_deg, min(self.max_steering_deg, value)) for name, value in wheel_angles.items()}

        # Publish only on significant change
        for name in self.wheel_modules:
            new_angle = wheel_angles[name]
            new_rpm = rpm[name]
            if self.should_publish(name, new_angle, new_rpm):
                cmd = Float64MultiArray(data=[float(new_rpm), float(new_angle)])
                self.wheel_pubs[name].publish(cmd)
                self.last_steer_cmd[name] = new_angle
                self.last_rpm_cmd[name] = new_rpm

    def display_status(self):
        self.get_logger().info(
            f"Mode: {self.mode_name(self.current_mode)} | "
            f"FB angles: FL={self.actual_angle['front_left']:.1f}° FR={self.actual_angle['front_right']:.1f}° "
            f"RL={self.actual_angle['rear_left']:.1f}° RR={self.actual_angle['rear_right']:.1f}°"
        )

def main(args=None):
    rclpy.init(args=args)
    node = FourWSController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()