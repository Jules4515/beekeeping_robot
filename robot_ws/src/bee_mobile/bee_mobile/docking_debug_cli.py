""" #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import threading
import sys
import math

class OdomPositionController(Node):
    def __init__(self):
        super().__init__('odom_position_controller')
        
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        self.aruco_frame = 'aruco_marker_91'
        
        # --- Limites de Vitesse Logique (Échelons) ---
        self.v_max = 0.07  # Vitesse lineaire logique max (sera propulsée par Swerve)
        self.w_max = 0.09  # Vitesse angulaire logique max
        
        # Gains Proportionnels (Pour réduire l'intensité de l'échelon si on est très proche)
        self.kp_pos = 1.2
        self.kp_yaw = 1.5
        
        # --- Paramètres du Contrôleur Pulsé (Step & Measure) ---
        self.pulse_duration = 0.25  # Durée de l'injection de vitesse (secondes)
        self.wait_duration = 0.40   # Temps d'arrêt total pour stabiliser l'odométrie (secondes)
        
        # État interne du pulseur
        self.micro_state = 'MEASURE' # 'MEASURE', 'PULSING', 'WAITING'
        self.state_start_time = self.get_clock().now()
        self.latched_cmd = Twist()   # Commande figée pendant l'impulsion
        self.macro_state_str = "ATTENTE"
        
        self.target_odom = None
        self.lock = threading.Lock()
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_docking', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Boucle d'asservissement a 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Odom Position Controller [MODE DISCRET/PULSÉ] initialisé.")
        self.get_logger().info(f"Impulsion: {self.pulse_duration}s | Freinage: {self.wait_duration}s")
        self.get_logger().info("Tapez 'aruco' dans ce terminal pour demarrer l'approche.")
        
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def get_transform(self, parent, child, log_error=False):
        try:
            trans = self.tf_buffer.lookup_transform(
                parent, 
                child, 
                rclpy.time.Time(), 
                timeout=Duration(seconds=0.1)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return [x, y, yaw]
        except TransformException as e:
            if log_error:
                self.get_logger().error(f"Echec de la liaison TF : {e}")
            return None

    def set_target(self, dx, dy, dyaw):
        current_pose = self.get_transform(self.odom_frame, self.base_frame, log_error=True)
        if current_pose is None: return

        x0, y0, theta0 = current_pose
        
        target_x = x0 + (dx * math.cos(theta0) - dy * math.sin(theta0))
        target_y = y0 + (dx * math.sin(theta0) + dy * math.cos(theta0))
        target_theta = self.normalize_angle(theta0 + dyaw)
        
        with self.lock:
            self.target_odom = [target_x, target_y, target_theta]
            # Réinitialise la machine d'états du pulseur
            self.micro_state = 'MEASURE'
            
        self.get_logger().info(f"CIBLE ABSOLUE FIGEE -> X: {target_x:.3f}m, Y: {target_y:.3f}m, Angle: {math.degrees(target_theta):.1f}°")

    def input_loop(self):
        while rclpy.ok():
            try:
                line = input("\n[COMMANDE] > ").strip()
                if not line: continue
                
                if line.lower() == 'aruco':
                    rel_pose = self.get_transform(self.base_frame, self.aruco_frame, log_error=True)
                    if rel_pose:
                        dx, dy, dyaw = rel_pose
                        dyaw_corrected = self.normalize_angle(dyaw + math.pi)
                        self.get_logger().info(f"Tag detecte : Dist={dx:.3f}m, Lateral={dy:.3f}m, Angle={math.degrees(dyaw_corrected):.1f}°")
                        self.set_target(dx - 0.25, dy, dyaw_corrected)
            except Exception as e:
                pass

    def control_loop(self):
        with self.lock:
            if self.target_odom is None:
                return
            target_x, target_y, target_theta = self.target_odom

        now = self.get_clock().now()
        elapsed_sec = (now - self.state_start_time).nanoseconds / 1e9

        # =========================================================================
        # PHASE 1 : FREINAGE & STABILISATION (WAITING)
        # =========================================================================
        if self.micro_state == 'WAITING':
            self.cmd_pub.publish(Twist()) # Force l'arrêt absolu
            if elapsed_sec >= self.wait_duration:
                self.micro_state = 'MEASURE' # Le robot est statique, on peut remesurer
            return

        # =========================================================================
        # PHASE 2 : EXÉCUTION DE L'IMPULSION (PULSING)
        # =========================================================================
        if self.micro_state == 'PULSING':
            self.cmd_pub.publish(self.latched_cmd) # Maintient l'échelon figé
            if elapsed_sec >= self.pulse_duration:
                self.micro_state = 'WAITING'
                self.state_start_time = now
                self.cmd_pub.publish(Twist()) # Coupe instantanément
            return

        # =========================================================================
        # PHASE 3 : MESURE STATIQUE & DÉCISION (MEASURE)
        # =========================================================================
        current_pose = self.get_transform(self.odom_frame, self.base_frame)
        if current_pose is None: return
        x, y, theta = current_pose

        err_x_odom = target_x - x
        err_y_odom = target_y - y
        err_theta = self.normalize_angle(target_theta - theta)

        err_x_local = err_x_odom * math.cos(-theta) - err_y_odom * math.sin(-theta)
        err_y_local = err_x_odom * math.sin(-theta) + err_y_odom * math.cos(-theta)

        TOL_ANGLE = math.radians(1.5)
        TOL_Y = 0.02
        TOL_X_FINAL = 0.05

        cmd = Twist()
        
        # --- MACHINE D'ETATS SEQUENTIELLE ---
        if abs(err_theta) > TOL_ANGLE:
            self.macro_state_str = "1/3 : CORRECTION ANGLE"
            cmd.angular.z = max(min(self.kp_yaw * err_theta, self.w_max), -self.w_max)

        elif abs(err_y_local) > TOL_Y:
            self.macro_state_str = "2/3 : ALIGNEMENT LATERAL (Y)"
            cmd.linear.y = max(min(self.kp_pos * err_y_local, self.v_max), -self.v_max)

        else:
            if err_x_local < TOL_X_FINAL:
                self.macro_state_str = "3/3 : ZONE ARRET ATTEINTE"
                with self.lock:
                    self.target_odom = None
                self.cmd_pub.publish(Twist())
                self.get_logger().info("Amarrage odométrique terminé avec succes.")
                return
            else:
                self.macro_state_str = "3/3 : APPROCHE RECTILIGNE"
                cmd.linear.x = max(min(self.kp_pos * err_x_local, self.v_max), -self.v_max)

        # Mémorisation de l'échelon et déclenchement de l'impulsion
        self.latched_cmd = cmd
        self.micro_state = 'PULSING'
        self.state_start_time = now

        # Affichage (uniquement au moment où l'on décide la nouvelle commande)
        self.get_logger().info(
            f"\n=== NOUVELLE IMPULSION ({self.pulse_duration}s) ==="
            f"\nETAT MACRO | {self.macro_state_str}"
            f"\nERREURS    | Devant: {err_x_local:.3f}m | Cote: {err_y_local * 100.0:.1f}cm | Cap: {math.degrees(err_theta):.1f}°"
            f"\nECHELON    | Vx: {cmd.linear.x:.3f} | Vy: {cmd.linear.y:.3f} | Wz: {cmd.angular.z:.3f}"
            f"\n===================================",
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdomPositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() """

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import threading
import sys
import math

class OdomPositionController(Node):
    def __init__(self):
        super().__init__('odom_position_controller')
        
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        self.aruco_frame = 'aruco_marker_91'
        
        # --- Limites de Vitesse Logique (Échelons) ---
        self.v_max = 0.06  
        self.w_max = 0.30  
        
        self.kp_pos = 1.2
        self.kp_yaw = 1.5
        
        # --- Paramètres du Contrôleur Pulsé Séquentiel ---
        self.pre_steer_duration = 0.40 # Temps laissé aux servomoteurs pour tourner les roues
        self.pulse_duration = 0.25     # Durée d'injection de la puissance motrice
        self.wait_duration = 0.40      # Temps de stabilisation des suspensions et de l'odométrie
        
        # État interne du pulseur
        self.micro_state = 'MEASURE' # 'MEASURE', 'PRE_STEER', 'PULSING', 'WAITING'
        self.state_start_time = self.get_clock().now()
        
        self.latched_cmd = Twist()       # La vraie commande de puissance
        self.latched_ghost_cmd = Twist() # La commande fantôme pour orienter les roues
        self.macro_state_str = "ATTENTE"
        
        self.target_odom = None
        self.lock = threading.Lock()
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_docking', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Odom Controller [MODE DISCRET AVEC PRE-STEERING].")
        self.get_logger().info("Tapez 'aruco' dans ce terminal pour demarrer l'approche.")
        
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def get_transform(self, parent, child, log_error=False):
        try:
            trans = self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return [x, y, yaw]
        except TransformException as e:
            if log_error: self.get_logger().error(f"Echec TF : {e}")
            return None

    def set_target(self, dx, dy, dyaw):
        current_pose = self.get_transform(self.odom_frame, self.base_frame, log_error=True)
        if current_pose is None: return

        x0, y0, theta0 = current_pose
        target_x = x0 + (dx * math.cos(theta0) - dy * math.sin(theta0))
        target_y = y0 + (dx * math.sin(theta0) + dy * math.cos(theta0))
        target_theta = self.normalize_angle(theta0 + dyaw)
        
        with self.lock:
            self.target_odom = [target_x, target_y, target_theta]
            self.micro_state = 'MEASURE'
            
        self.get_logger().info(f"CIBLE FIGEE -> X: {target_x:.3f}m, Y: {target_y:.3f}m, Angle: {math.degrees(target_theta):.1f}°")

    def input_loop(self):
        while rclpy.ok():
            try:
                line = input("\n[COMMANDE] > ").strip()
                if not line: continue
                if line.lower() == 'aruco':
                    rel_pose = self.get_transform(self.base_frame, self.aruco_frame, log_error=True)
                    if rel_pose:
                        dx, dy, dyaw = rel_pose
                        dyaw_corrected = self.normalize_angle(dyaw + math.pi)
                        self.set_target(dx - 0.25, dy, dyaw_corrected)
            except Exception: pass

    def generate_ghost_cmd(self, real_cmd):
        """
        Génère une commande directionnelle pure d'amplitude 0.002.
        Sert à réveiller le calcul d'angle de Swerve sans déclencher les moteurs.
        """
        ghost = Twist()
        V_GHOST = 0.002
        
        lin_norm = math.hypot(real_cmd.linear.x, real_cmd.linear.y)
        if lin_norm > 0:
            ghost.linear.x = (real_cmd.linear.x / lin_norm) * V_GHOST
            ghost.linear.y = (real_cmd.linear.y / lin_norm) * V_GHOST
            
        if abs(real_cmd.angular.z) > 0:
            ghost.angular.z = math.copysign(V_GHOST, real_cmd.angular.z)
            
        return ghost

    def control_loop(self):
        with self.lock:
            if self.target_odom is None: return
            target_x, target_y, target_theta = self.target_odom

        now = self.get_clock().now()
        elapsed_sec = (now - self.state_start_time).nanoseconds / 1e9

        # =========================================================================
        # PHASE 4 : FREINAGE & STABILISATION
        # =========================================================================
        if self.micro_state == 'WAITING':
            self.cmd_pub.publish(Twist()) # Force le zéro absolu
            if elapsed_sec >= self.wait_duration:
                self.micro_state = 'MEASURE'
            return

        # =========================================================================
        # PHASE 3 : EXÉCUTION DE L'IMPULSION (TRACTION)
        # =========================================================================
        if self.micro_state == 'PULSING':
            self.cmd_pub.publish(self.latched_cmd)
            if elapsed_sec >= self.pulse_duration:
                self.micro_state = 'WAITING'
                self.state_start_time = now
                self.cmd_pub.publish(Twist())
            return

        # =========================================================================
        # PHASE 2 : PRÉ-ORIENTATION DES ROUES (GHOST CMD)
        # =========================================================================
        if self.micro_state == 'PRE_STEER':
            self.cmd_pub.publish(self.latched_ghost_cmd)
            if elapsed_sec >= self.pre_steer_duration:
                self.micro_state = 'PULSING'
                self.state_start_time = now
            return

        # =========================================================================
        # PHASE 1 : MESURE STATIQUE & DÉCISION
        # =========================================================================
        current_pose = self.get_transform(self.odom_frame, self.base_frame)
        if current_pose is None: return
        x, y, theta = current_pose

        err_x_odom = target_x - x
        err_y_odom = target_y - y
        err_theta = self.normalize_angle(target_theta - theta)

        err_x_local = err_x_odom * math.cos(-theta) - err_y_odom * math.sin(-theta)
        err_y_local = err_x_odom * math.sin(-theta) + err_y_odom * math.cos(-theta)

        TOL_ANGLE = math.radians(1.5)
        TOL_Y = 0.02
        TOL_X_FINAL = 0.05

        cmd = Twist()
        
        # MACHINE D'ÉTATS SÉQUENTIELLE
        if abs(err_theta) > TOL_ANGLE:
            self.macro_state_str = "1/3 : CORRECTION ANGLE"
            cmd.angular.z = max(min(self.kp_yaw * err_theta, self.w_max), -self.w_max)
        elif abs(err_y_local) > TOL_Y:
            self.macro_state_str = "2/3 : ALIGNEMENT LATERAL"
            cmd.linear.y = max(min(self.kp_pos * err_y_local, self.v_max), -self.v_max)
        else:
            if err_x_local < TOL_X_FINAL:
                self.macro_state_str = "3/3 : ZONE ARRET ATTEINTE"
                with self.lock:
                    self.target_odom = None
                self.cmd_pub.publish(Twist())
                self.get_logger().info("Amarrage terminé avec succes.")
                return
            else:
                self.macro_state_str = "3/3 : APPROCHE RECTILIGNE"
                cmd.linear.x = max(min(self.kp_pos * err_x_local, self.v_max), -self.v_max)

        # Mémorisation des échelons
        self.latched_cmd = cmd
        self.latched_ghost_cmd = self.generate_ghost_cmd(cmd)

        # Bascule dans le micro-cycle de tir
        self.micro_state = 'PRE_STEER'
        self.state_start_time = now

        self.get_logger().info(
            f"\n=== CYCLE DE TIR (PreSteer:{self.pre_steer_duration}s -> Pulse:{self.pulse_duration}s) ==="
            f"\nETAT MACRO | {self.macro_state_str}"
            f"\nERREURS    | Devant: {err_x_local:.3f}m | Cote: {err_y_local * 100.0:.1f}cm | Cap: {math.degrees(err_theta):.1f}°"
            f"\nECHELON    | Vx: {cmd.linear.x:.3f} | Vy: {cmd.linear.y:.3f} | Wz: {cmd.angular.z:.3f}"
            f"\n=======================================================",
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdomPositionController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()