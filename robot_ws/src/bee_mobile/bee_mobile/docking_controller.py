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
            self.wheel_pubs[name] = self.create_publisher(Float64MultiArray, f'mobile/wheel_{name}/motor_speed', 10)
            self.create_subscription(Float64MultiArray, f'mobile/wheel_{name}/encoder_angle', lambda msg, n=name: self.encoder_callback(msg, n), 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- Deux boucles parallèles synchronisées à 20 Hz ---
        self.dt_s = 0.1
        self.timer = self.create_timer(self.dt_s, self.control_loop)
        self.cam_timer = self.create_timer(self.dt_s, self.camera_update_loop)
        
        self.get_logger().info("DOCKING CONTROLLER [ACTIVE] - Algorithme de Rollback Sécurisé")
        
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
                        self.micro_state = 'WAITING'
                        self.tag_perdu_recemment = False
                        self.rollback_autorise = True
                        self.last_tag_time = self.get_clock().now()
                    self.get_logger().info("==> VISUAL SERVOING ENABLED (20Hz) <==")
                elif line.lower() == 'stop':
                    with self.lock:
                        self.visual_servoing_active = False
                        self.target_odom = None
                    self.get_logger().info("==> EMERGENCY STOP <==")
            except Exception:
                pass

    def camera_update_loop(self):
        """Background 20Hz task to handle perception and target creation."""
        if not self.visual_servoing_active:
            return

        # 1. SAMPLE AND HOLD : Interdiction de lire la caméra pendant l'impulsion mécanique
        if self.micro_state == 'PULSING':
            return

        now = self.get_clock().now()
        pose_tag = self.get_tag_quaternion(self.base_frame, self.aruco_frame)
        
        if pose_tag:
            self.last_tag_time = now
            self.tag_perdu_recemment = False  # Reset du flag d'effondrement visuel
            
            x_tag, y_tag, z_tag, qx, qy, qz, qw = pose_tag
            
            # Extraction géométrique de la normale du plan (Axe Z de la ruche)
            vec_z_x = 2.0 * (qx * qz + qw * qy)
            vec_z_y = 2.0 * (qy * qz - qw * qx)
            tag_heading = math.atan2(vec_z_y, vec_z_x)
            
            dtheta_target = self.normalize_angle(tag_heading + math.pi)
            dx_target = x_tag - 1.056 - 0.25  # Camera offset + 25cm de garde de sécurité
            dy_target = y_tag
            
            # Reconstruction de la cible odométrique globale stable
            current_pose = self.get_transform(self.odom_frame, self.base_frame)
            if current_pose:
                x0, y0, theta0 = current_pose
                
                tx = x0 + (dx_target * math.cos(theta0) - dy_target * math.sin(theta0))
                ty = y0 + (dx_target * math.sin(theta0) + dy_target * math.cos(theta0))
                tt = self.normalize_angle(theta0 + dtheta_target)
                
                with self.lock:
                    self.target_odom = [tx, ty, tt]
        else:
            # 2. GESTION DE LA PERTE DU TAG (Rollback vs Odom Fallback)
            elapsed_since_last_seen = (now - self.last_tag_time).nanoseconds / 1e9
            
            # Si le tag disparaît brutalement à l'arrêt, et qu'on a le droit de rollback
            if elapsed_since_last_seen > 0.40 and not self.tag_perdu_recemment:
                if self.rollback_autorise:
                    self.tag_perdu_recemment = True
                    self.micro_state = 'ROLLBACK'
                    self.get_logger().warn("=> TAG PERDU ! Déclenchement du Rollback d'annulation.")
                else:
                    # Si l'anti-rollback est actif, on passe silencieusement en navigation odométrique pure (aveugle)
                    self.tag_perdu_recemment = True
                    self.get_logger().info("[FALLBACK] Perte naturelle du tag en approche finale. Maintien sur l'odométrie.")

    def apply_stiction_mapping(self, logical_speed):
        """
        Piecewise kinetic mapping combining square root and linear interpolation.
        Dynamic calculation of coefficients compensates for asymmetrical mechanical backlash.
        Note: Recalculating A, B, and m per call introduces minimal CPU overhead, 
        acceptable at 20Hz but could be a bottleneck if scaled to >1kHz loop rates.
        """
        V_MAX = 0.67
        V_MIN_NAV2 = 0.10
        V_MIN_DROITE = 0.30
        V_MIN_GAUCHE = 0.32
        
        abs_val = abs(logical_speed)
        
        if abs_val < 0.005: 
            return 0.0
        
        sign = math.copysign(1.0, logical_speed)
        
        # Select stiction threshold based on target direction
        v_min_moteur = V_MIN_GAUCHE if sign > 0 else V_MIN_DROITE
        
        # Dynamic calculation of piecewise coefficients
        m = (V_MAX - v_min_moteur) / (V_MAX - V_MIN_NAV2)
        A = (2.0 * (v_min_moteur - m * V_MIN_NAV2)) / math.sqrt(V_MIN_NAV2)
        B = 2.0 * m - (v_min_moteur / V_MIN_NAV2)
        
        if abs_val <= V_MIN_NAV2:
            mapped = sign * (A * math.sqrt(abs_val) + B * abs_val)
        else:
            mapped = sign * (m * (abs_val - V_MIN_NAV2) + v_min_moteur)
            
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
        """20Hz Main Kinematic Sequence."""
        with self.lock:
            if self.target_odom is None: 
                return
            target_x, target_y, target_theta = self.target_odom

        now = self.get_clock().now()
        elapsed_sec = (now - self.state_start_time).nanoseconds / 1e9

        ideal_angles = {}
        ideal_speeds = {}

        # Acquisition de l'odométrie pour l'affichage et les calculs d'erreurs
        current_pose = self.get_transform(self.odom_frame, self.base_frame)
        if current_pose is None: return
        x, y, theta = current_pose

        err_x_odom = target_x - x
        err_y_odom = target_y - y
        err_theta = self.normalize_angle(target_theta - theta)

        # Transformation de l'erreur odométrique globale en repère local robot
        err_x_local = err_x_odom * math.cos(-theta) - err_y_odom * math.sin(-theta)
        err_y_local = err_x_odom * math.sin(-theta) + err_y_odom * math.cos(-theta)

        # ---------------------------------------------------------
        # ÉTAPE SPÉCIALE : LE ROLLBACK D'URGENCE
        # ---------------------------------------------------------
        if self.micro_state == 'ROLLBACK':
            phase_str = "BACKTRACKING : ANNULATION DU DERNIER À-COUP"
            
            # Inversion stricte du dernier vecteur mémorisé avant la perte du tag
            inv_vx = -self.dernier_pulse_cmd[0]
            inv_vy = -self.dernier_pulse_cmd[1]
            inv_wz = -self.dernier_pulse_cmd[2]
            
            # Allocation des angles selon la nature de la commande inversée
            for name, config in self.wheels.items():
                if abs(inv_wz) > 0.0:
                    ideal_angles[name] = math.atan2(config['x'], -config['y'])
                elif abs(inv_vy) > 0.0:
                    ideal_angles[name] = math.radians(60.0) if inv_vy > 0 else math.radians(-60.0)
                else:
                    ideal_angles[name] = 0.0

            # Injection directe dans les filtres exponentiels pour exécuter un contre-pulse amorti
            self.filtered_vx = (self.alpha_v * inv_vx) + ((1.0 - self.alpha_v) * self.filtered_vx)
            self.filtered_vy = (self.alpha_v * inv_vy) + ((1.0 - self.alpha_v) * self.filtered_vy)
            self.filtered_wz = (self.alpha_wz * inv_wz) + ((1.0 - self.alpha_wz) * self.filtered_wz)
            
            for name, config in self.wheels.items():
                wz_comp = math.hypot(config['x'], config['y']) * self.filtered_wz
                vy_comp = self.filtered_vy / math.sin(math.radians(60.0))
                vx_comp = self.filtered_vx
                ideal_speeds[name] = wz_comp + vy_comp + vx_comp

            wheels_ready = self._check_wheels_ready(ideal_angles)
            
            if wheels_ready and elapsed_sec >= self.pulse_duration:
                # Une fois le contre-pulse achevé, on repasse en attente forcée pour chercher le tag
                self.micro_state = 'WAITING'
                self.state_start_time = now
                self.filtered_vx, self.filtered_vy, self.filtered_wz = 0.0, 0.0, 0.0
                
            self._apply_hardware(ideal_angles, ideal_speeds, phase_str, wheels_ready, current_pose=(x, y, theta), local_errors=(err_x_local, err_y_local, err_theta))
            return

        # ---------------------------------------------------------
        # EXÉCUTION DU PROTOCOLE PULSÉ STANDARDS
        # ---------------------------------------------------------
        if self.micro_state == 'WAITING':
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
                self.state_start_time = now 
                self.filtered_vx, self.filtered_vy, self.filtered_wz = 0.0, 0.0, 0.0
                status_str = " (AWAITING ALIGNMENT)"
            else:
                if elapsed_sec >= self.pulse_duration:
                    self.micro_state = 'WAITING'
                    self.state_start_time = now
                    self.filtered_vx, self.filtered_vy, self.filtered_wz = 0.0, 0.0, 0.0
                    status_str = " (BRAKING)"
                else:
                    self.filtered_vx = (self.alpha_v * self.latched_target_vx) + ((1.0 - self.alpha_v) * self.filtered_vx)
                    self.filtered_vy = (self.alpha_v * self.latched_target_vy) + ((1.0 - self.alpha_v) * self.filtered_vy)
                    self.filtered_wz = (self.alpha_wz * self.latched_target_wz) + ((1.0 - self.alpha_wz) * self.filtered_wz)
                    status_str = f" (PULSE {elapsed_sec:.2f}/{self.pulse_duration}s)"

            for name, config in self.wheels.items():
                wz_component = math.hypot(config['x'], config['y']) * self.filtered_wz
                vy_component = self.filtered_vy / math.sin(math.radians(60.0))
                vx_component = self.filtered_vx
                ideal_speeds[name] = wz_component + vy_component + vx_component

            self._apply_hardware(ideal_angles, ideal_speeds, self.latched_phase_str + status_str, wheels_ready, current_pose=(x, y, theta), local_errors=(err_x_local, err_y_local, err_theta))            
            return

        # ---------------------------------------------------------
        # MACHINE D'ÉTATS DYNAMIQUE (Calcul des consignes logiques)
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
            # --- BLOC D'APPROCHE FINALE EXCLUSIVE ---
            # Puisque Theta et Y sont dans les tolérances, nous verrouillons l'anti-rollback
            self.rollback_autorise = False 
            
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

        # Enregistrement des données de pulse pour le cycle cinématique bas niveau
        self.latched_ideal_angles = ideal_angles.copy()
        self.latched_target_wz = target_wz
        self.latched_target_vy = target_vy
        self.latched_target_vx = target_vx
        self.latched_phase_str = phase_str
        
        # Sauvegarde historique pour la fonction rollback
        self.dernier_pulse_cmd = [target_vx, target_vy, target_wz]
        
        self.micro_state = 'PULSING'
        self.state_start_time = now

        wheels_ready = self._check_wheels_ready(ideal_angles)
        self._apply_hardware(ideal_angles, {k: 0.0 for k in self.wheels.keys()}, phase_str + " (PREPARING)", wheels_ready, current_pose=(x, y, theta), local_errors=(err_x_local, err_y_local, err_theta))

    def _apply_hardware(self, base_angles, base_speeds, phase_str, wheels_ready, current_pose=None, local_errors=None):
        MAX_STEER_RAD_S = math.radians(60.0)
        log_wheel_lines = []

        rx, ry, rtheta = current_pose if current_pose else (0.0, 0.0, 0.0)
        ex, ey, etheta = local_errors if local_errors else (0.0, 0.0, 0.0)
        
        with self.lock:
            tx, ty, ttheta = self.target_odom if self.target_odom else (0.0, 0.0, 0.0)

        for name, config in self.wheels.items():
            target_ang = base_angles.get(name, config['last_angle'])
            target_spd = base_speeds.get(name, 0.0)
            
            if target_ang > (math.pi / 2.0):
                target_ang -= math.pi
                target_spd = -target_spd
            elif target_ang < -(math.pi / 2.0):
                target_ang += math.pi
                target_spd = -target_spd

            err_steer = self.normalize_angle(target_ang - config['last_angle'])
            max_step_steer = MAX_STEER_RAD_S * self.dt_s
            if abs(err_steer) > max_step_steer:
                cmd_angle = config['last_angle'] + math.copysign(max_step_steer, err_steer)
            else:
                cmd_angle = target_ang
            cmd_angle = max(min(cmd_angle, math.radians(80.0)), -math.radians(80.0))

            final_spd_target = target_spd if wheels_ready else 0.0
            physical_speed = self.apply_stiction_mapping(final_spd_target)

            config['last_angle'] = cmd_angle
            config['last_logical_speed'] = final_spd_target
            
            rpm_final = (physical_speed * 60.0) / (2.0 * math.pi * self.wheel_radius) * config['dir']
            self.wheel_pubs[name].publish(Float64MultiArray(data=[float(rpm_final), float(math.degrees(cmd_angle))]))

            display_name = name.replace('front_left', 'F.Left ').replace('front_right', 'F.Right').replace('rear_left', 'R.Left ').replace('rear_right', 'R.Right')
            log_wheel_lines.append(
                f"  [{display_name}] Target: {math.degrees(cmd_angle):>6.1f}° | R: {config['current_angle']:>6.1f}° | RPM: {rpm_final:>5.1f}"
            )

        wheels_formatted = "\n".join(log_wheel_lines)
        
        self.get_logger().info(
            f"\n=======================================================\n"
            f" STATUS   | {phase_str}\n"
            f" TRACTION | {'ACTIVE' if wheels_ready else 'LOCKED (Awaiting alignment)'}\n"
            f" AR-LOCK  | {'DÉSACTIVÉ (Sécurité active)' if self.rollback_autorise else 'VERROUILLÉ (Approche finale)'}\n"
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