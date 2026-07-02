#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, NavSatFix
from std_msgs.msg import Int8
from action_msgs.srv import CancelGoal
import os
import threading

class MuxJoystick(Node):
    """
    Reads a joystick and publishes /cmd_vel_joy for twist_mux.
    Uses dedicated buttons (A, B, X, Y) to lock specific movement axes.
    Integrated: Speed profiles, Nav2 Cancellation, and GPS Waypoint saving.
    """
    def __init__(self):
        super().__init__('mux_joystick')

        # Axis mapping
        self.declare_parameter('speed_axis', 1)
        self.declare_parameter('steer_axis', 0)
        self.declare_parameter('rotate_axis', 3)
        self.declare_parameter('dpad_x_axis', 6) # D-Pad Left/Right
        self.declare_parameter('dpad_y_axis', 7) # D-Pad Up/Down
        self.declare_parameter('deadband', 0.1)

        # Base Speed limits
        self.declare_parameter('max_linear_speed_ms', 0.15)
        self.declare_parameter('max_angular_speed_rads', 0.50)

        # Explicit button mapping
        self.declare_parameter('btn_straight', 0)        # A button
        self.declare_parameter('btn_zeroturn', 1)        # B button
        self.declare_parameter('btn_holonome', 2)        # X button
        self.declare_parameter('btn_crab', 3)            # Y button
        self.declare_parameter('btn_deadman', 5)         # R2/RT axis or button
        self.declare_parameter('btn_cancel_nav', 6)      # L2/LT button
        
        # GPS Save Path
        self.declare_parameter('yaml_path', os.path.expanduser('~/waypoints_GPS.yaml'))

        # Fetch parameters
        self.speed_axis = self.get_parameter('speed_axis').value
        self.steer_axis = self.get_parameter('steer_axis').value
        self.rotate_axis = self.get_parameter('rotate_axis').value
        self.dpad_x_axis = self.get_parameter('dpad_x_axis').value
        self.dpad_y_axis = self.get_parameter('dpad_y_axis').value
        self.deadband = self.get_parameter('deadband').value
        
        self.base_max_lin = self.get_parameter('max_linear_speed_ms').value
        self.base_max_ang = self.get_parameter('max_angular_speed_rads').value

        self.btn_straight = self.get_parameter('btn_straight').value
        self.btn_zeroturn = self.get_parameter('btn_zeroturn').value
        self.btn_holonome = self.get_parameter('btn_holonome').value
        self.btn_crab = self.get_parameter('btn_crab').value
        self.btn_deadman = self.get_parameter('btn_deadman').value
        self.btn_cancel_nav = self.get_parameter('btn_cancel_nav').value
        self.yaml_path = self.get_parameter('yaml_path').value

        # Speed Profiles
        self.speed_multiplier = 1.0 # Default (Speed 1)
        self.waypoint_counter = 1

        # Exponential Smoothing Filter State
        self.declare_parameter('alpha_filter', 0.10)
        self.declare_parameter('alpha_filter_wz', 0.05)
        self.alpha = self.get_parameter('alpha_filter').value
        self.alpha_wz = self.get_parameter('alpha_filter_wz').value

        self.filtered_joy_x = 0.0
        self.filtered_joy_y = 0.0
        self.filtered_joy_z = 0.0

        # State tracking (Debouncing)
        self.was_deadman_pressed = False
        self.last_dpad_x = 0.0
        self.last_dpad_y = 0.0
        self.last_l2 = 0
        self.last_buttons = []
        self.current_mode = 0 

        # Thread-safe GPS storage
        self.gps_lock = threading.Lock()
        self.latest_fix = None

        # Publishers / Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.mode_pub = self.create_publisher(Int8, '/joystick_control_mode', 10)
        
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.fix_sub = self.create_subscription(NavSatFix, '/fix', self.fix_callback, 10)
        
        # Action Client for Nav2 Cancel
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

    def fix_callback(self, msg):
        with self.gps_lock:
            self.latest_fix = msg

    def save_gps_waypoint(self):
        with self.gps_lock:
            if not self.latest_fix:
                self.get_logger().error("Impossible de sauvegarder: Aucun signal /fix reçu.")
                return
            lat = self.latest_fix.latitude
            lon = self.latest_fix.longitude

        file_exists = os.path.exists(self.yaml_path)
        is_empty = not file_exists or os.path.getsize(self.yaml_path) == 0

        try:
            with open(self.yaml_path, 'a') as f:
                if is_empty:
                    f.write("waypoints_GPS:\n\n")
                
                f.write(f"- name : WP_{self.waypoint_counter}\n")
                f.write(f"  latitude : {lat:.7f}\n")
                f.write(f"  longitude : {lon:.7f}\n")
                f.write(f"  yaw : 0.0\n")
                f.write(f"  wait_time : 0.0\n\n")
                
            self.get_logger().info(f"Point sauvegardé : WP_{self.waypoint_counter} -> {lat:.7f}, {lon:.7f}")
            self.waypoint_counter += 1
        except IOError as e:
            self.get_logger().error(f"Erreur d'écriture YAML: {e}")

    def cancel_nav2_goal(self):
        self.get_logger().warn("Annulation de l'objectif Nav2 demandée (L2).")
        if not self.cancel_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error("Service Nav2 indisponible.")
            return

        req = CancelGoal.Request()
        future = self.cancel_client.call_async(req)
        future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        try:
            res = future.result()
            if res.return_code == 0:
                self.get_logger().info("Nav2 stoppé avec succès.")
        except Exception as e:
            self.get_logger().error(f"Échec de l'annulation: {e}")

    def joy_callback(self, msg):
        # Read raw standard axes
        joy_x = msg.axes[self.speed_axis]
        joy_y = msg.axes[self.steer_axis]
        joy_z = msg.axes[self.rotate_axis]
        
        # Read D-Pad and Triggers
        dpad_x = msg.axes[self.dpad_x_axis] if len(msg.axes) > self.dpad_x_axis else 0.0
        dpad_y = msg.axes[self.dpad_y_axis] if len(msg.axes) > self.dpad_y_axis else 0.0
        l2_pressed = msg.buttons[self.btn_cancel_nav] if len(msg.buttons) > self.btn_cancel_nav else 0

        # Apply deadband
        if abs(joy_x) < self.deadband: joy_x = 0.0
        if abs(joy_y) < self.deadband: joy_y = 0.0
        if abs(joy_z) < self.deadband: joy_z = 0.0

        # --- D-PAD LOGIC: SPEED & GPS ---
        if dpad_y == 1.0 and self.last_dpad_y != 1.0:
            self.speed_multiplier = 1.0
            self.get_logger().info("Vitesse 1 (Lente)")
        elif dpad_x == -1.0 and self.last_dpad_x != -1.0:
            self.speed_multiplier = 2.0
            self.get_logger().info("Vitesse 2 (Moyenne)")
        elif dpad_y == -1.0 and self.last_dpad_y != -1.0:
            self.speed_multiplier = 3.0
            self.get_logger().info("Vitesse 3 (Rapide)")
            
        if dpad_x == 1.0 and self.last_dpad_x != 1.0:
            self.save_gps_waypoint()

        self.last_dpad_x = dpad_x
        self.last_dpad_y = dpad_y

        # --- L2 LOGIC: CANCEL NAV2 ---
        if l2_pressed == 1 and self.last_l2 == 0:
            self.cancel_nav2_goal()
        self.last_l2 = l2_pressed

        # Low-Pass Filter Math
        self.filtered_joy_x = (self.alpha * joy_x) + ((1.0 - self.alpha) * self.filtered_joy_x)
        self.filtered_joy_y = (self.alpha * joy_y) + ((1.0 - self.alpha) * self.filtered_joy_y)
        self.filtered_joy_z = (self.alpha_wz * joy_z) + ((1.0 - self.alpha_wz) * self.filtered_joy_z)

        # Mode Selection
        buttons = msg.buttons
        if not self.last_buttons:
            self.last_buttons = [0] * len(buttons)

        max_btn_index = max(self.btn_straight, self.btn_crab, self.btn_zeroturn, self.btn_holonome)
        if len(buttons) > max_btn_index:
            if buttons[self.btn_holonome] and not self.last_buttons[self.btn_holonome]:
                self.current_mode = 0
                self.get_logger().info("Mode: Holonome")
            elif buttons[self.btn_straight] and not self.last_buttons[self.btn_straight]:
                self.current_mode = 1
                self.get_logger().info("Mode: Straight")
            elif buttons[self.btn_crab] and not self.last_buttons[self.btn_crab]:
                self.current_mode = 2
                self.get_logger().info("Mode: Crab")
            elif buttons[self.btn_zeroturn] and not self.last_buttons[self.btn_zeroturn]:
                self.current_mode = 3
                self.get_logger().info("Mode: Zero Turn")

        self.last_buttons = list(buttons)

        # Apply Current Speed Profile
        current_max_lin = self.base_max_lin * self.speed_multiplier
        current_max_ang = self.base_max_ang * self.speed_multiplier

        # Deadman switch evaluation (Hardware axis < 0.0)
        deadman_pressed = (msg.axes[self.btn_deadman] < 0.0) 

        if deadman_pressed:
            mode_msg = Int8()
            mode_msg.data = self.current_mode
            self.mode_pub.publish(mode_msg)

        twist = Twist()

        if deadman_pressed:
            if self.current_mode == 1:
                twist.linear.x = self.filtered_joy_x * current_max_lin
                twist.linear.y = 0.0
                twist.angular.z = self.filtered_joy_z * current_max_ang
            elif self.current_mode == 2:
                twist.linear.x = self.filtered_joy_x * current_max_lin
                twist.linear.y = self.filtered_joy_y * current_max_lin
                twist.angular.z = 0.0
            elif self.current_mode == 3:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = self.filtered_joy_z * current_max_ang
            else:
                twist.linear.x = self.filtered_joy_x * current_max_lin
                twist.linear.y = self.filtered_joy_y * current_max_lin
                twist.angular.z = self.filtered_joy_z * current_max_ang
        
            self.cmd_vel_pub.publish(twist)
            self.was_deadman_pressed = True

        else:
            if self.was_deadman_pressed:
                self.cmd_vel_pub.publish(twist)
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
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()