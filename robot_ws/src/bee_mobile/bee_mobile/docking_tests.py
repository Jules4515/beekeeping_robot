#!/usr/bin/env python3
# 
# Docking Tests Utilities
#
# !! Work in progress !!, file used by the docking controller and docking test nodes.
#
# This node publishes wheel commands for straight, crab, and zero-turn motion,
# verifies TF2 transformations and timing behavior, and provides smooth-pulse
# tests for validating the docking controller hardware and control logic.
# 

import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from bee_mobile.docking_math import send_pulse, get_universal_transform, send_pulse_smooth

class DockingTests(Node):
    def __init__(self):
        super().__init__('docking_tests')

        # ==========================================
        # TEST SELECTION
        # 0 = IDLE
        # 1 = STRAIGHT PULSE (0 deg)
        # 2 = CRAB PULSE (+-60 deg)
        # 3 = ZERO TURN PULSE (Rotational)
        # 4 = TF: aruco_marker_91 -> camera_link
        # 5 = TF: camera_link -> base_link
        # 6 = TF: base_link -> base_footprint
        # 7 = TF: base_footprint -> odom
        # ==========================================
        self.ACTIVE_TEST = 0
        self.pulse_direction = 1.0  # Direction multiplier (1.0 = left/forward, -1.0 = right/backward)

        # --- Frames ---
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        self.aruco_frame = 'aruco_marker_91'
        
        # --- Strict Logic Limits (Pulse Amplitude) ---
        self.v_pulse = 0.30
        self.w_pulse = 0.30
        self.wheel_radius = 0.215
        
        # --- State Machine Tolerances ---
        self.tol_angle = math.radians(0.5)
        self.tol_y     = 0.05
        self.tol_x     = 0.05
        
        # --- Pulse & Filter Configuration ---
        self.pulse_duration = 0.8
        self.wait_duration = 1.0
        self.micro_state = 'WAITING'
        self.state_start_time = self.get_clock().now()
        
        # Fast exponential filters to reach pulse saturation in 0.25s
        self.alpha_v = 0.10
        self.alpha_wz = 0.05
        
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_wz = 0.0
        
        # Latched logic targets for the current pulse
        self.latched_target_vx = 0.0
        self.latched_target_vy = 0.0
        self.latched_target_wz = 0.0
        self.latched_ideal_angles = {}
        self.latched_phase_str = "WAITING"
        
        # --- Tag History and Management Variables ---
        self.last_tag_time = self.get_clock().now()
        self.tag_perdu_recemment = False
        self.rollback_autorise = True  # Safety trigger
        
        # Store the last generated velocity vector [vx, vy, wz]
        self.dernier_pulse_cmd = [0.0, 0.0, 0.0]
        
        self.wheels = {
            'front_left':  {'x': 0.48,  'y': 0.4150, 'dir': 1.0, 'enc_dir': 1.0,  'current_speed_ms': 0.0, 'current_angle_deg': 0.0, 'last_speed_ms': 0.0, 'last_angle_deg': 0.0},
            'front_right': {'x': 0.48,  'y': -0.4150, 'dir': 1.0, 'enc_dir': -1.0, 'current_speed_ms': 0.0, 'current_angle_deg': 0.0, 'last_speed_ms': 0.0, 'last_angle_deg': 0.0},
            'rear_left':   {'x': -0.48, 'y': 0.4150,  'dir': 1.0, 'enc_dir': 1.0,  'current_speed_ms': 0.0, 'current_angle_deg': 0.0, 'last_speed_ms': 0.0, 'last_angle_deg': 0.0},
            'rear_right':  {'x': -0.48, 'y': -0.4150, 'dir': 1.0, 'enc_dir': -1.0, 'current_speed_ms': 0.0, 'current_angle_deg': 0.0, 'last_speed_ms': 0.0, 'last_angle_deg': 0.0},
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

        self.perception_timer = self.create_timer(0.033, self.execute_tf_buffer_tests) # 30 Hz

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Pulse timer tracking
        self.pulse_start_time_s = 0.0
        self.timer = self.create_timer(0.05, self.control_loop)

        self.input_thread = threading.Thread(target=self.keyboard_listener_loop, daemon=True)
        self.input_thread.start()

    def encoder_callback(self, msg, wheel_name):
        with self.lock:
            if len(msg.data) >= 2:
                current_rpm = float(msg.data[0]) * self.wheels[wheel_name]['enc_dir']
                # Convert RPM to m/s.
                current_speed_ms = (current_rpm * 2.0 * math.pi * self.wheel_radius) / 60.0
                self.wheels[wheel_name]['current_speed_ms'] = current_speed_ms
                self.wheels[wheel_name]['current_angle_deg'] = float(msg.data[1])
            else:
                self.get_logger().warn(
                    f"Malformed encoder message for {wheel_name}: expected 2 values, got {len(msg.data)}",
                    once=True
                )

    def control_loop(self):
        if self.ACTIVE_TEST == 0:
            return
            
        with self.lock:
            if self.ACTIVE_TEST in [1, 2, 3]:
                self.execute_kinematic_test()
            elif self.ACTIVE_TEST in [4, 5, 6, 7]:
                self.execute_tf_test()
            elif self.ACTIVE_TEST in [8, 9]:
                self.execute_tf_buffer_tests()
            elif self.ACTIVE_TEST in [10, 11]:
                self.execute_smooth_kinematic_test()

    def execute_kinematic_test(self):
        target_vx, target_vy, target_wz = 0.0, 0.0, 0.0
        
        if self.ACTIVE_TEST == 1:
            target_vx = self.v_pulse * self.pulse_direction
        elif self.ACTIVE_TEST == 2:
            target_vy = self.v_pulse * self.pulse_direction
        elif self.ACTIVE_TEST == 3:
            target_wz = self.w_pulse * self.pulse_direction

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

        fl_ang_deg = self.wheels['front_left']['current_angle_deg']
        fl_speed_ms = self.wheels['front_left']['current_speed_ms']
        self.get_logger().info(
            f"[{self.micro_state}] t_pulse: {elapsed_s:.2f}s | "
            f"Cmd [Vx:{self.filtered_vx:.2f}, Vy:{self.filtered_vy:.2f}, Wz:{self.filtered_wz:.2f}] | "
            f"FL_deg: {fl_ang_deg:.1f}°, FL_ms: {fl_speed_ms:.2f}",
            throttle_duration_sec=0.20  # Display every 200 ms.
        )

    def execute_tf_test(self):
        mappings = {
            4: ('camera_link', self.aruco_frame),
            5: (self.base_frame, 'camera_link', ),
            6: (self.base_frame, self.aruco_frame),
            7: (self.odom_frame, self.aruco_frame)
            #7: (self.odom_frame, 'base_footprint')
        }
        
        parent, child = mappings[self.ACTIVE_TEST]
        tf_data = get_universal_transform(self.tf_buffer, parent, child)
        
        if tf_data:
            # --- GEOMETRIC GIMBAL-LOCK FIX ---
            # If the target is an ArUco tag, ignore the quaternion yaw
            # (optical noise) and calculate the actual heading (2D direction
            # vector) from the parent-frame origin.
            if 'aruco_marker' in child:
                stable_yaw_rad = math.atan2(tf_data['y_m'], tf_data['x_m'])
                tf_data['yaw_rad'] = stable_yaw_rad
            # ----------------------------------------

        if tf_data:
            self.get_logger().info(
                f"[{parent} -> {child}] "
                f"X: {tf_data['x_m']:.3f}m, Y: {tf_data['y_m']:.3f}m, Z: {tf_data['z_m']:.3f}m | "
                f"Yaw: {math.degrees(tf_data['yaw_rad']):.3f}deg"
            )

    def execute_tf_buffer_tests(self):
        """
        Substitutes the old async perception callback. 
        Executes TF2 buffer verification synchronously within the 20Hz timer.
        """
        now_ros = self.get_clock().now()

        if self.ACTIVE_TEST == 8:
            # Test 8: Check immediate snapshot availability.
            # tf_exact requests the snapshot at the current nanosecond.
            tf_exact = get_universal_transform(self.tf_buffer, self.odom_frame, self.aruco_frame, now_ros)
            # tf_latest requests the most recent available transform (Time 0).
            tf_latest = get_universal_transform(self.tf_buffer, self.odom_frame, self.aruco_frame)
            
            self.get_logger().info(
                f"[TEST 8] ArUco -> Odom | "
                f"Strict Current Instant: {'OK' if tf_exact else 'FAIL'} | "
                f"Last Known (Time 0): {'OK' if tf_latest else 'FAIL'}",
                throttle_duration_sec=0.5
            )

        elif self.ACTIVE_TEST == 9:
            # Test 9: Evaluate blocking and temporal lookup behavior.
            start_wait = time.time()
            try:
                # Ask TF2 to actively wait up to 50 ms for the requested transform.
                self.tf_buffer.lookup_transform(
                    self.odom_frame, 
                    self.aruco_frame, 
                    now_ros, 
                    rclpy.duration.Duration(seconds=0.05)
                )
                delay_ms = (time.time() - start_wait) * 1000.0
                self.get_logger().info(f"[TEST 9] TF synchronized after waiting {delay_ms:.1f} ms")
            except Exception as e:
                self.get_logger().error(
                    f"[TEST 9] No synchronized transform after waiting 50 ms. Error: {str(e)}",
                    throttle_duration_sec=0.5
                )

    def execute_smooth_kinematic_test(self):
        mode = 1 if self.ACTIVE_TEST == 10 else 2

        if self.micro_state == 'WAITING':
            self.latched_target_vx = 0.4
            self.latched_target_vy = 0.0
            self.latched_target_wz = 0.0
            self.pulse_start_time_s = time.time()
            self.micro_state = 'WAITING_WHEEL_ORIENTATION'
            self.get_logger().info(f"[TEST {self.ACTIVE_TEST}] Starting smooth profile (Mode {mode}). Vmax=0.35, T=1.0s")

        elapsed_s = 0.0
        if self.micro_state == 'SENDING_PULSE':
            elapsed_s = time.time() - self.pulse_start_time_s

        new_state, self.filtered_vx, self.filtered_vy, self.filtered_wz = send_pulse_smooth(
            self.micro_state, elapsed_s, 0.8,
            self.latched_target_vx, self.latched_target_vy, self.latched_target_wz,
            self.filtered_vx, self.filtered_vy, self.filtered_wz,
            0.45, 0.45, mode, 0.05,                      # dt=0.05 - must match self.timer (create_timer(0.05, ...))
            self.wheels, self.wheel_pubs, self.wheel_radius
        )

        if self.micro_state == 'WAITING_WHEEL_ORIENTATION' and new_state == 'SENDING_PULSE':
            self.pulse_start_time_s = time.time()
            self.get_logger().info(f"[TEST {self.ACTIVE_TEST}] Wheels aligned. Acceleration phase.")

        if self.micro_state == 'SENDING_PULSE' and new_state == 'WAITING':
            self.filtered_vx, self.filtered_vy, self.filtered_wz = 0.0, 0.0, 0.0
            self.get_logger().info(f"[TEST {self.ACTIVE_TEST}] Smooth pulse complete. Auto-stop.")
            self.ACTIVE_TEST = 0

        self.micro_state = new_state

    def keyboard_listener_loop(self):
        """Blocking loop running in a separate thread to detect keyboard commands."""
        menu = "\n=== CMD === | 0:IDLE | 1-3:KINEMATICS | 4-7:TF | 8:RACE CHECK | 9:LATENCY | [Enter]:Restart ==="
        print(menu)
        
        while rclpy.ok():
            try:
                cmd = input("\nCommande > ").strip().lower()
                
                with self.lock:
                    # Safety: block new commands while a pulse is running.
                    if self.micro_state != 'WAITING' and self.ACTIVE_TEST in [1, 2, 3]:
                        self.get_logger().warning(f"Ignored: Robot is moving | State: {self.micro_state}")
                        continue
                        
                    if cmd != "":
                        try:
                            # Separate the leading digits (test number) from the
                            # optional trailing direction letter ('b' or 'r').
                            digits = ''
                            suffix = ''
                            for ch in cmd:
                                if ch.isdigit():
                                    digits += ch
                                else:
                                    suffix += ch

                            mode = int(digits)
                            if not (0 <= mode <= 11):
                                raise ValueError

                            self.ACTIVE_TEST = mode
                            self.pulse_direction = 1.0

                            if suffix:
                                d = suffix[0]
                                if mode == 1 and d == 'b':
                                    self.pulse_direction = -1.0
                                elif mode in [2, 3] and d == 'r':
                                    self.pulse_direction = -1.0

                        except (ValueError, IndexError):
                            print("Syntax error. Valid examples: '1', '1b', '3r', '10', '11'")
                            continue
                            
                    # Apply software initialization for the selected test.
                    if self.ACTIVE_TEST in [1, 2, 3]:
                        # Clear the exponential-filter buffers for a clean start.
                        self.filtered_vx = 0.0
                        self.filtered_vy = 0.0
                        self.filtered_wz = 0.0
                        self.get_logger().info(f"--- RESTART PULSE | TEST: {self.ACTIVE_TEST} | DIR: {self.pulse_direction} ---")
                    elif self.ACTIVE_TEST in [4, 5, 6, 7]:
                        self.get_logger().info(f"--- CONTINUOUS TF READING (TEST {self.ACTIVE_TEST}) ---")
                    else:
                        self.get_logger().info("--- IDLE ---")

            except EOFError:
                break
            except Exception as e:
                self.get_logger().error(f"Keyboard thread error: {e}")
                break

def main(args=None):
    rclpy.init(args=args)
    node = DockingTests()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print(f"[INFO] [{node.get_name()}]: Shutdown requested by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()