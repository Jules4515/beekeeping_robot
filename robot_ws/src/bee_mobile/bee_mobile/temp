#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import threading
import math

class DirectDockingController(Node):
    def __init__(self):
        super().__init__('direct_docking_controller')
        
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        self.aruco_frame = 'aruco_marker_91'
        
        # --- Logical Limits ---
        self.v_pulse = 0.08  
        self.w_pulse = 0.08  
        self.wheel_radius = 0.215
        
        # --- Tolerances ---
        self.tol_angle = math.radians(0.5)
        self.tol_y     = 0.05
        self.tol_x     = 0.05
        
        self.kp_pos = 1.2
        self.kp_yaw = 1.5
        
        # --- Pulse & Filter Settings ---
        self.pulse_duration = 0.25
        self.wait_duration = 0.50
        self.micro_state = 'WAITING'
        self.state_start_time = self.get_clock().now()
        
        # Exponential filter constants (0.0 to 1.0)
        # Smaller = smoother but slower response
        self.alpha_v = 0.15
        self.alpha_wz = 0.10
        
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_wz = 0.0
        
        # Latched targets for the current pulse
        self.latched_target_vx = 0.0
        self.latched_target_vy = 0.0
        self.latched_target_wz = 0.0
        self.latched_ideal_angles = {}
        self.latched_phase_str = "WAITING"
        
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
            self.wheel_pubs[name] = self.create_publisher(Float64MultiArray, f'mobile/wheel_{name}/motor_speed', 10)
            self.create_subscription(Float64MultiArray, f'mobile/wheel_{name}/encoder_angle', lambda msg, n=name: self.encoder_callback(msg, n), 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- Timers (Sensor Fusion) ---
        # Low-level control loop
        frequency_control_loop_hz = 20
        self.dt_s = 1.0 / frequency_control_loop_hz
        self.timer = self.create_timer(self.dt_s, self.control_loop)
        
        # High-level camera update loop
        frequency_camera_loop_hz = frequency_control_loop_hz / 4.0
        self.cam_timer = self.create_timer(1.0 / frequency_camera_loop_hz, self.camera_update_loop)
        
        self.get_logger().info("DOCKING CONTROLLER [ACTIVE]")
        
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def encoder_callback(self, msg, wheel_name):
        if len(msg.data) >= 2:
            self.wheels[wheel_name]['current_angle'] = float(msg.data[1])

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def get_transform(self, parent, child, log_error=False):
        """2D Odom tracking"""
        try:
            trans = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.1))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return [x, y, yaw]
        except TransformException as e:
            if log_error: self.get_logger().error(f"TF Error (Odom): {e}")
            return None

    def get_tag_quaternion(self, parent, child, log_error=False):
        """3D TF for extracting normal vector Z"""
        try:
            trans = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.1))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            q = trans.transform.rotation
            return [x, y, z, q.x, q.y, q.z, q.w]
        except TransformException as e:
            if log_error: self.get_logger().error(f"TF Error (Tag): {e}")
            return None

    def input_loop(self):
        """Activates/Deactivates continuous visual servoing"""
        while rclpy.ok():
            try:
                line = input("\n[CMD] > ").strip()
                if line.lower() == 'aruco':
                    with self.lock:
                        self.visual_servoing_active = True
                        self.micro_state = 'MEASURE'
                    self.get_logger().info("==> VISUAL SERVOING ENABLED (5Hz) <==")
                elif line.lower() == 'stop':
                    with self.lock:
                        self.visual_servoing_active = False
                        self.target_odom = None
                    self.get_logger().info("==> EMERGENCY STOP <==")
            except Exception:
                pass

    def camera_update_loop(self):
        """Background 5Hz task to update the global target dynamically."""
        if not self.visual_servoing_active:
            return

        # =========================================================
        # FIX : SAMPLE AND HOLD (Anti-Latence)
        # =========================================================
        # 1. On interdit la lecture caméra si le robot est en mouvement.
        if self.micro_state != 'WAITING':
            return
            
        # 2. On attend 200ms APRÈS l'arrêt des moteurs pour laisser la 
        # structure mécanique se stabiliser et la caméra capturer une 
        # image nette qui correspond exactement à l'odométrie actuelle.
        now = self.get_clock().now()
        elapsed_sec = (now - self.state_start_time).nanoseconds / 1e9
        if elapsed_sec < 0.20:
            return

        pose_tag = self.get_tag_quaternion(self.base_frame, self.aruco_frame)
        if pose_tag:
            x_tag, y_tag, z_tag, qx, qy, qz, qw = pose_tag
            
            # Extract Tag's Z-axis (Normal vector) for parallelism
            vec_z_x = 2.0 * (qx * qz + qw * qy)
            vec_z_y = 2.0 * (qy * qz - qw * qx)
            tag_heading = math.atan2(vec_z_y, vec_z_x)
            
            # Target geometry (anti-parallel to tag)
            dtheta_target = self.normalize_angle(tag_heading + math.pi)
            dx_target = x_tag - 1.056 - 0.25  # Camera offset
            dy_target = y_tag
            
            # Project onto fixed odometry map
            current_pose = self.get_transform(self.odom_frame, self.base_frame)
            if current_pose:
                x0, y0, theta0 = current_pose
                
                tx = x0 + (dx_target * math.cos(theta0) - dy_target * math.sin(theta0))
                ty = y0 + (dx_target * math.sin(theta0) + dy_target * math.cos(theta0))
                tt = self.normalize_angle(theta0 + dtheta_target)
                
                with self.lock:
                    self.target_odom = [tx, ty, tt]

    def apply_stiction_mapping(self, logical_speed):
        """Maps logical speed to physical hardware limits to overcome stiction."""
        V_MAX = 0.67
        V_MIN_MOTEUR = 0.30
        V_MIN_NAV2 = 0.10
        abs_val = abs(logical_speed)
        
        if abs_val < 0.005: return 0.0
        
        sign = math.copysign(1.0, logical_speed)
        m = (V_MAX - V_MIN_MOTEUR) / (V_MAX - V_MIN_NAV2)
        A = (2.0 * (V_MIN_MOTEUR - m * V_MIN_NAV2)) / math.sqrt(V_MIN_NAV2)
        B = 2.0 * m - (V_MIN_MOTEUR / V_MIN_NAV2)
        
        if abs_val <= V_MIN_NAV2:
            mapped = sign * (A * math.sqrt(abs_val) + B * abs_val)
        else:
            mapped = sign * (m * (abs_val - V_MIN_NAV2) + V_MIN_MOTEUR)
            
        return max(min(mapped, V_MAX), -V_MAX)

    def _check_wheels_ready(self, base_angles):
        """Validates if all steering servos have reached their target."""
        TOLERANCE_STEERING_RAD = math.radians(5.0)
        max_servo_error = 0.0
        
        for name, config in self.wheels.items():
            if name not in base_angles: return False
            target_ang = base_angles[name]
            
            if target_ang > (math.pi / 2.0): target_ang -= math.pi
            elif target_ang < -(math.pi / 2.0): target_ang += math.pi
                
            phys_angle = math.radians(config['current_angle'])
            diff = target_ang - phys_angle
            err_ang = math.atan2(math.sin(diff), math.cos(diff))
            err_ang_alternate = math.atan2(math.sin(diff + math.pi), math.cos(diff + math.pi))
            
            best_err = min(abs(err_ang), abs(err_ang_alternate))
            if best_err > max_servo_error: max_servo_error = best_err
                
        return (max_servo_error <= TOLERANCE_STEERING_RAD)

    def control_loop(self):
        """20Hz sequence: Handles state evaluations, filters and motor pulsing."""
        with self.lock:
            if self.target_odom is None: return
            target_x, target_y, target_theta = self.target_odom

        now = self.get_clock().now()
        elapsed_sec = (now - self.state_start_time).nanoseconds / 1e9

        ideal_angles = {}
        ideal_speeds = {}

        # ---------------------------------------------------------
        # PHASE 0: Measurement
        # ---------------------------------------------------------
        current_pose = self.get_transform(self.odom_frame, self.base_frame)
        if current_pose is None: return
        x, y, theta = current_pose

        err_x_odom = target_x - x
        err_y_odom = target_y - y
        err_theta = self.normalize_angle(target_theta - theta)

        # Convert global error to local base_link error
        err_x_local = err_x_odom * math.cos(-theta) - err_y_odom * math.sin(-theta)
        err_y_local = err_x_odom * math.sin(-theta) + err_y_odom * math.cos(-theta)

        # ---------------------------------------------------------
        # PHASE 1: Execution (Pulsing or Waiting)
        # ---------------------------------------------------------
        if self.micro_state == 'WAITING':
            # Collapse filters and speed to absolute zero
            self.filtered_vx = 0.0
            self.filtered_vy = 0.0
            self.filtered_wz = 0.0
            
            for name in self.wheels.keys():
                ideal_angles[name] = self.latched_ideal_angles.get(name, 0.0)
                ideal_speeds[name] = 0.0
                
            if elapsed_sec >= self.wait_duration:
                self.micro_state = 'MEASURE'
            
            wheels_ready = self._check_wheels_ready(ideal_angles)
            self._apply_hardware(ideal_angles, ideal_speeds, self.latched_phase_str + " (BRAKING)", wheels_ready, current_pose=(x, y, theta), local_errors=(err_x_local, err_y_local, err_theta))
            return

        elif self.micro_state == 'PULSING':
            for name in self.wheels.keys():
                ideal_angles[name] = self.latched_ideal_angles.get(name, 0.0)
                
            wheels_ready = self._check_wheels_ready(ideal_angles)
            
            if not wheels_ready:
                # Timer freeze: Do not pulse if wheels are still turning
                self.state_start_time = now 
                self.filtered_vx = 0.0
                self.filtered_vy = 0.0
                self.filtered_wz = 0.0
                status_str = " (AWAITING ALIGNMENT)"
            else:
                if elapsed_sec >= self.pulse_duration:
                    self.micro_state = 'WAITING'
                    self.state_start_time = now
                    self.filtered_vx = 0.0
                    self.filtered_vy = 0.0
                    self.filtered_wz = 0.0
                    status_str = " (BRAKING)"
                else:
                    # Apply discrete Exponential Filter
                    self.filtered_vx = (self.alpha_v * self.latched_target_vx) + ((1.0 - self.alpha_v) * self.filtered_vx)
                    self.filtered_vy = (self.alpha_v * self.latched_target_vy) + ((1.0 - self.alpha_v) * self.filtered_vy)
                    self.filtered_wz = (self.alpha_wz * self.latched_target_wz) + ((1.0 - self.alpha_wz) * self.filtered_wz)
                    status_str = f" (PULSE {elapsed_sec:.2f}/{self.pulse_duration}s)"

            # Combine filtered speeds using Inverse Kinematics
            for name, config in self.wheels.items():
                wz_component = math.hypot(config['x'], config['y']) * self.filtered_wz
                vy_component = self.filtered_vy / math.sin(math.radians(60.0))
                vx_component = self.filtered_vx
                ideal_speeds[name] = wz_component + vy_component + vx_component

            self._apply_hardware(
                ideal_angles, 
                ideal_speeds, 
                self.latched_phase_str + status_str, 
                wheels_ready, 
                current_pose=(x, y, theta), 
                local_errors=(err_x_local, err_y_local, err_theta)
            )            
            return

        # ---------------------------------------------------------
        # PHASE 2: Dynamic State Machine (Constant Pulse Magnitude)
        # ---------------------------------------------------------

        phase_str = "STOP"
        target_wz = 0.0
        target_vy = 0.0
        target_vx = 0.0

        if abs(err_theta) > self.tol_angle:
            phase_str = "1/3: ROTATION (ZERO-TURN)"
            target_wz = math.copysign(self.w_pulse, err_theta)
            
            for name, config in self.wheels.items():
                ideal_angles[name] = math.atan2(config['x'], -config['y'])

        elif abs(err_y_local) > self.tol_y:
            phase_str = "2/3: LATERAL ALIGNMENT (CRAB ±60°)"
            crab_angle = math.radians(60.0) if err_y_local > 0 else math.radians(-60.0)
            target_vy = math.copysign(self.v_pulse, err_y_local)
            
            for name in self.wheels.keys():
                ideal_angles[name] = crab_angle

        else:
            if err_x_local < self.tol_x:
                phase_str = "3/3: TARGET REACHED"
                with self.lock: 
                    self.visual_servoing_active = False
                    self.target_odom = None
                for name in self.wheels.keys():
                    ideal_angles[name] = self.wheels[name]['last_angle']
                self.get_logger().info("Docking completed successfully.")
                self._apply_hardware(ideal_angles, {k: 0.0 for k in self.wheels.keys()}, phase_str, True, current_pose=(x, y, theta), local_errors=(err_x_local, err_y_local, err_theta))
                return
            else:
                phase_str = "3/3: FINAL APPROACH (0°)"
                target_vx = math.copysign(self.v_pulse, err_x_local)
                
                for name in self.wheels.keys():
                    ideal_angles[name] = 0.0

        # Latch targets to preserve stability during the incoming pulse
        self.latched_ideal_angles = ideal_angles.copy()
        self.latched_target_wz = target_wz
        self.latched_target_vy = target_vy
        self.latched_target_vx = target_vx
        self.latched_phase_str = phase_str
        
        self.micro_state = 'PULSING'
        self.state_start_time = now

        wheels_ready = self._check_wheels_ready(ideal_angles)
        self._apply_hardware(ideal_angles, {k: 0.0 for k in self.wheels.keys()}, phase_str + " (PREPARING)", wheels_ready, current_pose=(x, y, theta), local_errors=(err_x_local, err_y_local, err_theta))

    def _apply_hardware(self, base_angles, base_speeds, phase_str, wheels_ready, current_pose=None, local_errors=None):
        MAX_STEER_RAD_S = math.radians(60.0)
        log_wheel_lines = []

        # 1. Extraction des données de position et d'erreurs pour l'affichage
        rx, ry, rtheta = current_pose if current_pose else (0.0, 0.0, 0.0)
        ex, ey, etheta = local_errors if local_errors else (0.0, 0.0, 0.0)
        
        # Récupération de la dernière cible odométrique (la ruche modifiée par la caméra)
        with self.lock:
            tx, ty, ttheta = self.target_odom if self.target_odom else (0.0, 0.0, 0.0)

        for name, config in self.wheels.items():
            target_ang = base_angles.get(name, config['last_angle'])
            target_spd = base_speeds.get(name, 0.0)
            
            # Phase inversion logic for optimized servo path
            if target_ang > (math.pi / 2.0):
                target_ang -= math.pi
                target_spd = -target_spd
            elif target_ang < -(math.pi / 2.0):
                target_ang += math.pi
                target_spd = -target_spd

            # Steering safety slew rate
            err_steer = self.normalize_angle(target_ang - config['last_angle'])
            max_step_steer = MAX_STEER_RAD_S * self.dt_s
            if abs(err_steer) > max_step_steer:
                cmd_angle = config['last_angle'] + math.copysign(max_step_steer, err_steer)
            else:
                cmd_angle = target_ang
            cmd_angle = max(min(cmd_angle, math.radians(80.0)), -math.radians(80.0))

            # Hardware execution
            final_spd_target = target_spd if wheels_ready else 0.0
            physical_speed = self.apply_stiction_mapping(final_spd_target)

            config['last_angle'] = cmd_angle
            config['last_logical_speed'] = final_spd_target
            
            rpm_final = (physical_speed * 60.0) / (2.0 * math.pi * self.wheel_radius) * config['dir']
            self.wheel_pubs[name].publish(Float64MultiArray(data=[float(rpm_final), float(math.degrees(cmd_angle))]))

            # --- CORRECTION DES NOMS DES ROUES (Explicit Left/Right) ---
            # Aligne proprement les chaînes : [F.Left ], [F.Right], [R.Left ], [R.Right]
            display_name = name.replace('front_left', 'F.Left ').replace('front_right', 'F.Right').replace('rear_left', 'R.Left ').replace('rear_right', 'R.Right')
            log_wheel_lines.append(
                f"  [{display_name}] Target: {math.degrees(cmd_angle):>6.1f}° | R: {config['current_angle']:>6.1f}° | RPM: {rpm_final:>5.1f}"
            )

        wheels_formatted = "\n".join(log_wheel_lines)
        
        # --- LOG MONOLITHIQUE UNIQUE ---
        self.get_logger().info(
            f"\n=======================================================\n"
            f" STATUS   | {phase_str}\n"
            f" TRACTION | {'ACTIVE' if wheels_ready else 'LOCKED (Awaiting alignment)'}\n"
            f"--------------------- POSITIONS -----------------------\n"
            f" Robot    | X: {rx:>6.3f}m | Y: {ry:>6.3f}m | Yaw: {math.degrees(rtheta):>5.1f}°\n"
            f" Ruche    | X: {tx:>6.3f}m | Y: {ty:>6.3f}m | Yaw: {math.degrees(ttheta):>5.1f}°\n"
            f"---------------------- ERREURS ------------------------\n"
            f" Locales  | dX: {ex:>5.3f}m | dY: {ey*100:>5.1f}cm | dTheta: {math.degrees(etheta):>5.1f}°\n"
            f"--------------------- KINEMATICS ----------------------\n"
            f"{wheels_formatted}\n"
            f"=======================================================",
            throttle_duration_sec=0.5
        )

def main(args=None):
    rclpy.init(args=args)
    node = DirectDockingController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        for name, pub in node.wheel_pubs.items():
            pub.publish(Float64MultiArray(data=[0.0, 0.0]))
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()