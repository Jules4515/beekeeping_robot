#!/usr/bin/env python3
import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from bee_mobile.docking_math import send_pulse, get_universal_transform

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')

        # --- Frames ---
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        self.aruco_frame = 'aruco_marker_91'
        self.camera_frame = 'camera_link'
        
        # --- Limites Logiques Strictes ---
        self.v_pulse = 0.30
        self.w_pulse = 0.30
        self.wheel_radius_m = 0.215
        
        # --- Tolérances de la Machine d'États ---
        self.tol_theta_deg = 0.5
        self.tol_y_m     = 0.05
        self.tol_x_m     = 0.05
        
        # --- Configuration des Pulses & Filtres ---
        self.pulse_duration_s = 0.60
        self.wait_duration_s = 1.0
        
        self.alpha_v = 0.10
        self.alpha_wz = 0.05
        
        self.filtered_vx_ms = 0.0
        self.filtered_vy_ms = 0.0
        self.filtered_wz_rad_s = 0.0
        
        # --- État du Séquenceur ---
        self._state = 'WAITING'  # Variable privée, utiliser set_state() pour modifier
        self.wait_start_time_s = time.time()
        self.pulse_start_time_s = 0.0
        
        # Cibles logiques mémorisées
        self.latched_target_vx = 0.0
        self.latched_target_vy = 0.0
        self.latched_target_wz = 0.0
        self.last_pulse_cmd = [0.0, 0.0, 0.0]
        
        # --- Variables d'Historique et Perception ---
        self.target_odom = None
        self.last_tag_time_s = time.time()
        self.tag_lost_recently = False
        self.rollback_allowed = True
        
        self.lock = threading.Lock()
        
        self.wheels = {
            'front_left':  {'x': 0.48,  'y': 0.4150, 'dir': 1.0, 'enc_dir': 1.0,  'current_speed_ms': 0.0, 'current_angle_deg': 0.0, 'last_speed_ms': 0.0, 'last_angle_deg': 0.0},
            'front_right': {'x': 0.48,  'y': -0.4150, 'dir': 1.0, 'enc_dir': -1.0, 'current_speed_ms': 0.0, 'current_angle_deg': 0.0, 'last_speed_ms': 0.0, 'last_angle_deg': 0.0},
            'rear_left':   {'x': -0.48, 'y': 0.4150,  'dir': 1.0, 'enc_dir': 1.0,  'current_speed_ms': 0.0, 'current_angle_deg': 0.0, 'last_speed_ms': 0.0, 'last_angle_deg': 0.0},
            'rear_right':  {'x': -0.48, 'y': -0.4150, 'dir': 1.0, 'enc_dir': -1.0, 'current_speed_ms': 0.0, 'current_angle_deg': 0.0, 'last_speed_ms': 0.0, 'last_angle_deg': 0.0},
        }
        
        self.wheel_pubs = {}
        for name in self.wheels.keys():
            self.wheel_pubs[name] = self.create_publisher(Float64MultiArray, f'mobile/wheel_{name}/motor_speed', 10)
            self.create_subscription(Float64MultiArray, f'mobile/wheel_{name}/encoder_angle', lambda msg, n=name: self.encoder_callback(msg, n), 10)

        self.perception_timer = self.create_timer(0.066, self.perception_loop)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Process B: Synchronous 20Hz Control Loop
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Docking Controller Initialized. Waiting for initial conditions.")

    def set_state(self, new_state):
        """Centralizes state transitions to automatically generate debug logs."""
        if self._state != new_state:
            self.get_logger().info(f"[STATE TRANSITION] {self._state} ---> {new_state}")
            self._state = new_state

    def encoder_callback(self, msg, wheel_name):
        with self.lock:
            if len(msg.data) >= 2:
                current_rpm = float(msg.data[0]) * self.wheels[wheel_name]['enc_dir']
                self.wheels[wheel_name]['current_speed_ms'] = (current_rpm * 2.0 * math.pi * self.wheel_radius_m) / 60.0
                self.wheels[wheel_name]['current_angle_deg'] = float(msg.data[1])

    def perception_loop(self):
        """
        Process A: Interrogation directe de l'arbre TF2.
        STRICT SAMPLING: Mis à jour uniquement quand le robot est immobile.
        """
        with self.lock:
            if self._state != 'WAITING':
                return

        # 1. Interrogation de TF2 depuis le centre du châssis.
        tf_base_to_tag = get_universal_transform(self.tf_buffer, self.base_frame, self.aruco_frame)
        
        if not tf_base_to_tag:
            with self.lock:
                if self.target_odom is not None and (time.time() - self.last_tag_time_s) > 0.4:
                    self.tag_lost_recently = True
            return

        tf_odom_to_base = get_universal_transform(self.tf_buffer, self.odom_frame, self.base_frame)
        
        if tf_odom_to_base:
            with self.lock:
                robot_yaw = tf_odom_to_base['yaw_rad']
                
                # 2. Projection cartésienne (X = Avant, Y = Gauche)
                local_x = tf_base_to_tag['x_m']
                local_y = tf_base_to_tag['y_m']
                
                target_x_m = tf_odom_to_base['x_m'] + (local_x * math.cos(robot_yaw) - local_y * math.sin(robot_yaw))
                target_y_m = tf_odom_to_base['y_m'] + (local_x * math.sin(robot_yaw) + local_y * math.cos(robot_yaw))

                # 3. FIX VECTORIEL: Calcul du cap via la normale de la ruche
                qx = tf_base_to_tag['qx']
                qy = tf_base_to_tag['qy']
                qz = tf_base_to_tag['qz']
                qw = tf_base_to_tag['qw']

                # Le vecteur Z du tag correspond à 2*(qx*qz + qw*qy) pour X et 2*(qy*qz - qw*qx) pour Y
                nx = 2.0 * (qx * qz + qw * qy)
                ny = 2.0 * (qy * qz - qw * qx)

                # La ruche regarde vers l'extérieur (nx, ny). Le robot doit lui faire face (-nx, -ny).
                tag_yaw_local = math.atan2(-ny, -nx)
                
                target_yaw_rad = robot_yaw + tag_yaw_local
                target_yaw_rad = math.atan2(math.sin(target_yaw_rad), math.cos(target_yaw_rad)) # Normalisation

                if self.target_odom is None or self.tag_lost_recently:
                    self.get_logger().info(f"[PERCEPTION] Target Acquired (Abs Odom): X={target_x_m:.2f}, Y={target_y_m:.2f}, Yaw={math.degrees(target_yaw_rad):.2f}°")

                self.target_odom = {
                    'x_m': target_x_m,
                    'y_m': target_y_m,
                    'yaw_rad': target_yaw_rad
                }
                
                self.last_tag_time_s = time.time()
                self.tag_lost_recently = False
                self.rollback_allowed = True

    def control_loop(self):
        with self.lock:
            # Sécurité prioritaire absolue
            if self.tag_lost_recently and self.rollback_allowed and self._state not in ['ROLLBACK', 'SENDING_PULSE']:
                self.get_logger().error("[CONTROL] Triggering Rollback due to recent tag loss.")
                self.set_state('ROLLBACK')

            # Routage vers le bon handler d'état
            if self._state == 'WAITING':
                self._handle_waiting_state()
            
            elif self._state == 'EVALUATE_ERRORS':
                self._handle_evaluation_state()
                
            elif self._state in ['WAITING_WHEEL_ORIENTATION', 'SENDING_PULSE']:
                self._handle_pulse_execution()

            elif self._state == 'ROLLBACK':
                self._handle_rollback()

    def _handle_waiting_state(self):
        # Immobilisation des moteurs
        send_pulse(
            self._state, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            self.alpha_v, self.alpha_wz, self.wheels, self.wheel_pubs, self.wheel_radius_m
        )
        
        elapsed = time.time() - self.wait_start_time_s
        if elapsed >= self.wait_duration_s:
            if self.target_odom is not None:
                self.set_state('EVALUATE_ERRORS')
            else:
                self.get_logger().info("[WAITING] No target to track yet. Idling...", throttle_duration_sec=2.0)

    def _handle_evaluation_state(self):
        robot_pose = get_universal_transform(self.tf_buffer, self.odom_frame, self.base_frame)
        if not robot_pose:
            self.get_logger().warn("[EVALUATION] TF Error: Cannot fetch robot pose. Falling back to WAITING.", throttle_duration_sec=1.0)
            self.wait_start_time_s = time.time()
            self.set_state('WAITING')
            return

        dx = self.target_odom['x_m'] - robot_pose['x_m']
        dy = self.target_odom['y_m'] - robot_pose['y_m']
        
        theta = robot_pose['yaw_rad']
        err_x_local = dx * math.cos(theta) + dy * math.sin(theta)
        err_y_local = -dx * math.sin(theta) + dy * math.cos(theta)
        
        # Le robot cherche à s'aligner parallèlement à la normale de la ruche
        diff = self.target_odom['yaw_rad'] - theta
        err_theta_rad = math.atan2(math.sin(diff), math.cos(diff))
        err_theta_deg = math.degrees(err_theta_rad)

        self.latched_target_vx, self.latched_target_vy, self.latched_target_wz = 0.0, 0.0, 0.0
        
        self.get_logger().info(f"[EVALUATION] Errors -> X: {err_x_local:.3f}m | Y: {err_y_local:.3f}m | Theta: {err_theta_deg:.2f}°")

        if abs(err_theta_deg) > self.tol_theta_deg:
            self.latched_target_wz = math.copysign(self.w_pulse, err_theta_rad)
            self.get_logger().info(f"[DECISION] Priority 1: Zero-Turn Mode (Wz={self.latched_target_wz})")
        elif abs(err_y_local) > self.tol_y_m:
            self.latched_target_vy = math.copysign(self.v_pulse, err_y_local)
            self.get_logger().info(f"[DECISION] Priority 2: Crab Mode (Vy={self.latched_target_vy})")
        elif abs(err_x_local) > self.tol_x_m:
            self.latched_target_vx = math.copysign(self.v_pulse, err_x_local)
            self.get_logger().info(f"[DECISION] Priority 3: Straight Mode (Vx={self.latched_target_vx})")
        else:
            self.get_logger().info("====================================")
            self.get_logger().info("==== DOCKING SEQUENCE SUCCESSFUL ===")
            self.get_logger().info("====================================")
            self.wait_start_time_s = time.time()
            self.set_state('WAITING')
            return

        self.last_pulse_cmd = [self.latched_target_vx, self.latched_target_vy, self.latched_target_wz]
        self.set_state('WAITING_WHEEL_ORIENTATION')

    def _handle_pulse_execution(self):
        elapsed_s = 0.0
        if self._state == 'SENDING_PULSE':
            elapsed_s = time.time() - self.pulse_start_time_s

        new_micro_state, self.filtered_vx_ms, self.filtered_vy_ms, self.filtered_wz_rad_s = send_pulse(
            self._state, elapsed_s, self.pulse_duration_s,
            self.latched_target_vx, self.latched_target_vy, self.latched_target_wz,
            self.filtered_vx_ms, self.filtered_vy_ms, self.filtered_wz_rad_s,
            self.alpha_v, self.alpha_wz,
            self.wheels, self.wheel_pubs, self.wheel_radius_m
        )

        if self._state == 'WAITING_WHEEL_ORIENTATION' and new_micro_state == 'SENDING_PULSE':
            self.pulse_start_time_s = time.time()
            self.get_logger().info("[PULSE] Wheels aligned. Injecting thrust vector.")
            
        if self._state == 'SENDING_PULSE' and new_micro_state == 'WAITING':
            self.filtered_vx_ms, self.filtered_vy_ms, self.filtered_wz_rad_s = 0.0, 0.0, 0.0
            self.wait_start_time_s = time.time()
            self.get_logger().info("[PULSE] Pulse completed. Braking.")
            
        self.set_state(new_micro_state)

    def _handle_rollback(self):
        self.get_logger().warn(f"[ROLLBACK] Inverting last vector: {self.last_pulse_cmd}")
        self.latched_target_vx = -self.last_pulse_cmd[0]
        self.latched_target_vy = -self.last_pulse_cmd[1]
        self.latched_target_wz = -self.last_pulse_cmd[2]
        
        self.rollback_allowed = False
        self.set_state('WAITING_WHEEL_ORIENTATION')

def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
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