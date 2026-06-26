#!/usr/bin/env python3
import math
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from docking_math import send_pulse, get_universal_transform

class DockingTests(Node):
    def __init__(self):
        super().__init__('docking_tests')

        # ==========================================
        # TEST SELECTION
        # 0 = IDLE
        # 1 = STRAIGHT PULSE (0 deg)
        # 2 = CRAB PULSE (+-60 deg)
        # 3 = ZERO TURN PULSE (Rotational)
        # 4 = TF: aruco_marker_91 -> camera_link_optical
        # 5 = TF: camera_link_optical -> base_link
        # 6 = TF: base_link -> base_footprint
        # 7 = TF: base_footprint -> odom
        # ==========================================
        self.ACTIVE_TEST = 1  

        # --- Frames ---
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        self.aruco_frame = 'aruco_marker_91'
        
        # --- Limites Logiques Strictes (Amplitude de Pulse Bang-Bang) ---
        self.v_pulse = 0.20
        self.w_pulse = 0.20
        self.wheel_radius = 0.215
        
        # --- Tolérances de la Machine d'États ---
        self.tol_angle = math.radians(0.5)
        self.tol_y     = 0.05
        self.tol_x     = 0.05
        
        # --- Configuration des Pulses & Filtres ---
        self.pulse_duration = 0.25
        self.wait_duration = 0.50
        self.micro_state = 'WAITING'
        self.state_start_time = self.get_clock().now()
        
        # Filtres exponentiels rapides pour saturer le pulse en 0.25s
        self.alpha_v = 0.10
        self.alpha_wz = 0.05
        
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_wz = 0.0
        
        # Cibles logiques mémorisées pour l'impulsion en cours
        self.latched_target_vx = 0.0
        self.latched_target_vy = 0.0
        self.latched_target_wz = 0.0
        self.latched_ideal_angles = {}
        self.latched_phase_str = "WAITING"
        
        # --- Variables d'Historique et de Gestion du Tag ---
        self.last_tag_time = self.get_clock().now()
        self.tag_perdu_recemment = False
        self.rollback_autorise = True  # Déclencheur de sécurité
        
        # Stockage du dernier mouvement vectoriel généré [vx, vy, wz]
        self.dernier_pulse_cmd = [0.0, 0.0, 0.0]
        
        self.wheels = {
            'front_left':  {'x': 0.48,  'y': 0.4150,  'dir': 1.0, 'enc_dir': 1.0,  'current_angle': 0.0, 'last_angle': 0.0, 'last_logical_speed': 0.0},
            'front_right': {'x': 0.48,  'y': -0.4150, 'dir': 1.0, 'enc_dir': -1.0, 'current_angle': 0.0, 'last_angle': 0.0, 'last_logical_speed': 0.0},
            'rear_left':   {'x': -0.48, 'y': 0.4150,  'dir': 1.0, 'enc_dir': 1.0,  'current_angle': 0.0, 'last_angle': 0.0, 'last_logical_speed': 0.0},
            'rear_right':  {'x': -0.48, 'y': -0.4150, 'dir': 1.0, 'enc_dir': -1.0, 'current_angle': 0.0, 'last_angle': 0.0, 'last_logical_speed': 0.0},
        }

        self.target_odom = None
        self.visual_servoing_active = False
        self.lock = threading.Lock()
        
        self.wheel_pubs = {}
        for name in self.wheels.keys():
            self.wheel_pubs[name] = self.create_publisher(
                Float64MultiArray, 
                f'mobile/wheel_{name}/motor_speed', 
                10
            )
            self.create_subscription(
                Float64MultiArray, 
                f'mobile/wheel_{name}/encoder_angle', 
                lambda msg, n=name: self.encoder_callback(msg, n), 
                10
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Pulse timer tracking
        self.pulse_start_time_s = 0.0
        self.timer = self.create_timer(0.05, self.control_loop)

    def encoder_callback(self, msg, wheel_name):
        with self.lock:
            # Application of hardware encoder direction mapping
            raw_angle = msg.data[0] * self.wheels[wheel_name]['enc_dir']
            self.wheels[wheel_name]['current_angle'] = raw_angle
            # Maintain backward compatibility with docking_math.py logic
            self.wheels[wheel_name]['current_angle_deg'] = raw_angle

    def control_loop(self):
        if self.ACTIVE_TEST == 0:
            return
            
        with self.lock:
            if self.ACTIVE_TEST in [1, 2, 3]:
                self.execute_kinematic_test()
            elif self.ACTIVE_TEST in [4, 5, 6, 7]:
                self.execute_tf_test()

    def execute_kinematic_test(self):
        target_vx, target_vy, target_wz = 0.0, 0.0, 0.0
        
        if self.ACTIVE_TEST == 1:
            target_vx = self.v_pulse
        elif self.ACTIVE_TEST == 2:
            target_vy = self.v_pulse
        elif self.ACTIVE_TEST == 3:
            target_wz = self.w_pulse

        elapsed_s = 0.0
        if self.micro_state == 'SENDING_PULSE':
            elapsed_s = time.time() - self.pulse_start_time_s

        new_state, self.filtered_vx, self.filtered_vy, self.filtered_wz = send_pulse(
            state=self.micro_state,
            elapsed_s=elapsed_s,
            pulse_duration_s=self.pulse_duration,
            target_vx_ms=target_vx,
            target_vy_ms=target_vy,
            target_wz_rad_s=target_wz,
            filtered_vx_ms=self.filtered_vx,
            filtered_vy_ms=self.filtered_vy,
            filtered_wz_rad_s=self.filtered_wz,
            alpha_v=self.alpha_v,
            alpha_wz=self.alpha_wz,
            wheels_config=self.wheels,
            wheel_pubs=self.wheel_pubs,
            wheel_radius_m=self.wheel_radius
        )

        if self.micro_state == 'WAITING_WHEEL_ORIENTATION' and new_state == 'SENDING_PULSE':
            self.pulse_start_time_s = time.time()
            
        if self.micro_state == 'SENDING_PULSE' and new_state == 'WAITING':
            self.ACTIVE_TEST = 0  # Disable to prevent infinite pulsing
            
        self.micro_state = new_state

    def execute_tf_test(self):
        mappings = {
            4: (self.aruco_frame, 'camera_link_optical'),
            5: ('camera_link_optical', self.base_frame),
            6: (self.base_frame, 'base_footprint'),
            7: ('base_footprint', self.odom_frame)
        }
        
        parent, child = mappings[self.ACTIVE_TEST]
        tf_data = get_universal_transform(self.tf_buffer, parent, child)
        
        if tf_data:
            self.get_logger().info(
                f"[{parent} -> {child}] "
                f"X: {tf_data['x_m']:.3f}m, Y: {tf_data['y_m']:.3f}m, Z: {tf_data['z_m']:.3f}m | "
                f"Yaw: {tf_data['yaw_rad']:.3f}rad"
            )

def main(args=None):
    rclpy.init(args=args)
    node = DockingTests()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()