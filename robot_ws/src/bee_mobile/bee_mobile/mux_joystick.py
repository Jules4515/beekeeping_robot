#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, NavSatFix, Imu
from std_msgs.msg import Int8
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import math
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

        # Speed profile values for the three joystick speed modes.
        # ROS2 parameters natively support flat float arrays, not nested arrays.
        self.declare_parameter(
            'speed_profiles',
            [
                0.15, 0.20,  # Slow mode: vx, wz
                0.45, 0.45,  # Normal mode: vx, wz
                0.70, 0.70,  # Fast mode: vx, wz
            ]
        )

        # Explicit button mapping
        self.declare_parameter('btn_straight', 0)        # A button
        self.declare_parameter('btn_zeroturn', 1)        # B button
        self.declare_parameter('btn_holonome', 2)        # X button
        self.declare_parameter('btn_crab', 3)            # Y button
        self.declare_parameter('btn_deadman', 5)         # R2/RT axis or button
        self.declare_parameter('btn_cancel_nav', 6)      # L2/LT button

        # Fetch parameters
        self.speed_axis = self.get_parameter('speed_axis').value
        self.steer_axis = self.get_parameter('steer_axis').value
        self.rotate_axis = self.get_parameter('rotate_axis').value
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
        self.speed_mode = 1  # Normal mode by default

        self.btn_straight = self.get_parameter('btn_straight').value
        self.btn_zeroturn = self.get_parameter('btn_zeroturn').value
        self.btn_holonome = self.get_parameter('btn_holonome').value
        self.btn_crab = self.get_parameter('btn_crab').value
        self.btn_deadman = self.get_parameter('btn_deadman').value
        self.btn_cancel_nav = self.get_parameter('btn_cancel_nav').value

        # GPS Save Path
        pkg_share = get_package_share_directory('bee_mobile') # Vérifie que ce nom correspond bien à ton package
        self.yaml_path = os.path.join(pkg_share, 'config', 'waypoints_GPS.yaml')

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
        self.latest_yaw = None

        # Publishers / Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.mode_pub = self.create_publisher(Int8, '/joystick_control_mode', 10)
        
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.fix_sub = self.create_subscription(NavSatFix, '/fix', self.fix_callback, 10)
        self.heading_sub = self.create_subscription(Imu, '/heading_imu', self.imu_callback, 10)
        
        # Action Client for Nav2 Cancel
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

    def fix_callback(self, msg):
        with self.gps_lock:
            self.latest_fix = msg
    
    def imu_callback(self, msg):
        with self.gps_lock:
            self.latest_imu = msg

    def euler_from_quaternion(self, q):
        # Convert quaternion to euler angles (ZYX order)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
        cosp = math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
        pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def save_gps_waypoint(self):
        """
        Sauvegarde la position GPS courante et le cap IMU dans un fichier YAML.
        Le fichier est écrit dans config/ du package bee_mobile (répertoire installé).
        Si le fichier n'existe pas, il est créé avec l'en-tête waypoints_GPS.
        """
        with self.gps_lock:
            if not self.latest_fix:
                self.get_logger().error("Impossible de sauvegarder: aucun signal /fix reçu.")
                return
            lat = self.latest_fix.latitude
            lon = self.latest_fix.longitude

            _, _, yaw = self.euler_from_quaternion(self.latest_imu.orientation)

        try:
            # Crée le répertoire parent si besoin (ex : install/bee_mobile/share/bee_mobile/config/)
            os.makedirs(os.path.dirname(self.yaml_path), exist_ok=True)

            # Crée le fichier avec l'en-tête s'il n'existe pas encore
            if not os.path.exists(self.yaml_path):
                with open(self.yaml_path, 'w') as f:
                    f.write("waypoints_GPS:\n\n")

            # S'assure qu'il y a un saut de ligne avant d'ajouter un nouvel enregistrement
            with open(self.yaml_path, 'rb') as f:
                f.seek(0, os.SEEK_END)
                if f.tell() > 0:
                    f.seek(-1, os.SEEK_END)
                    last_char = f.read(1)
            if last_char != b'\n':
                with open(self.yaml_path, 'a') as f:
                    f.write('\n')

            # Écriture du waypoint
            with open(self.yaml_path, 'a') as f:
                f.write(f"- name      : WP_{self.waypoint_counter}\n")
                f.write(f"  latitude  : {lat:.7f}\n")
                f.write(f"  longitude : {lon:.7f}\n")
                f.write(f"  yaw       : {yaw:.2f}\n")   # degrés, depuis /heading_imu
                f.write(f"  wait_time : 0.0\n\n")

            self.get_logger().info(
                f"WP_{self.waypoint_counter} sauvegardé → "
                f"lat={lat:.7f}, lon={lon:.7f}, yaw={yaw:.1f}°"
            )
            self.waypoint_counter += 1

        except IOError as e:
            self.get_logger().error(f"Erreur d'écriture YAML : {e}")

    def cancel_nav2_goal(self):
        """
        Envoie une requête vide au service cancel_goal de l'action navigate_to_pose.
        Une requête vide (goals=[]) annule TOUS les objectifs en cours,
        ce qui est équivalent à :
        ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
        Le service n'est disponible que si Nav2 tourne — on vérifie avant d'appeler.
        """
        self.get_logger().warn("Annulation de l'objectif Nav2 demandée.")

        if not self.cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                "Service /navigate_to_pose/_action/cancel_goal indisponible. "
                "Nav2 est-il lancé ?"
            )
            return

        req = CancelGoal.Request()
        # req.goals est vide par défaut → annule tous les goals actifs
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
            self.speed_mode = 0
            self.get_logger().info(
                f"Slow Speed Mode 1 : vx={self.speed_profiles[0][0]:.2f} m/s, wz={self.speed_profiles[0][1]:.2f} rad/s"
            )
        elif dpad_x == -1.0 and self.last_dpad_x != -1.0:
            self.speed_mode = 1
            self.get_logger().info(
                f"Normal Speed Mode 2 : vx={self.speed_profiles[1][0]:.2f} m/s, wz={self.speed_profiles[1][1]:.2f} rad/s"
            )
        elif dpad_y == -1.0 and self.last_dpad_y != -1.0:
            self.speed_mode = 2
            self.get_logger().info(
                f"Fast Speed Mode 3 : vx={self.speed_profiles[2][0]:.2f} m/s, wz={self.speed_profiles[2][1]:.2f} rad/s"
            )
            
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
        current_max_lin = self.speed_profiles[self.speed_mode][0]
        current_max_ang = self.speed_profiles[self.speed_mode][1]

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