# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import threading
# import sys
# import math

# class DockingDebugCLI(Node):
#     def __init__(self):
#         super().__init__('docking_debug_cli')
        
#         # Le topic de sortie configuré pour ton multiplexeur
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_docking', 10)
        
#         # Paramètres de simulation
#         self.hz = 20.0
#         self.dt = 1.0 / self.hz
#         self.decay_rate = 0.10  # Décélération linéaire absolue (m/s² ou rad/s²)
        
#         # État courant des consignes [vx, vy, wz]
#         self.current_v = [0.0, 0.0, 0.0]
        
#         # Verrou de sécurité pour la mémoire partagée entre les threads
#         self.lock = threading.Lock()
        
#         # Boucle de publication à 20 Hz
#         self.timer = self.create_timer(self.dt, self.timer_callback)
        
#         self.get_logger().info("Docking Debug CLI Initialisé.")
#         self.get_logger().info("Format attendu : vx vy wz (ex: '0.30 0.0 0.0'). Tapez Ctrl+C pour quitter.")
        
#         # Lancement du thread de lecture du terminal en arrière-plan
#         self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
#         self.input_thread.start()

#     def input_loop(self):
#         """Thread secondaire : Bloque sur la fonction input() sans figer ROS 2."""
#         while rclpy.ok():
#             try:
#                 line = input("\n[Nouvelle Consigne] vx vy wz > ")
#                 parts = line.strip().split()
                
#                 if len(parts) == 3:
#                     with self.lock:
#                         self.current_v[0] = float(parts[0])
#                         self.current_v[1] = float(parts[1])
#                         self.current_v[2] = float(parts[2])
#                     print(f"--> INJECTION : vx={self.current_v[0]:.3f}, vy={self.current_v[1]:.3f}, wz={self.current_v[2]:.3f}")
#                 else:
#                     print("Erreur : Format invalide. Il faut exactement 3 nombres séparés par des espaces.")
#             except ValueError:
#                 print("Erreur : Les valeurs doivent être des nombres valides.")
#             except (EOFError, KeyboardInterrupt):
#                 # Géré proprement par le main()
#                 break

#     def timer_callback(self):
#         """Thread principal ROS 2 : Applique la décélération et publie le Twist."""
#         msg = Twist()
#         step = self.decay_rate * self.dt  # Quantité à soustraire à chaque cycle (0.005 à 20Hz)
        
#         with self.lock:
#             for i in range(3):
#                 # Si la vitesse est positive, on la réduit vers 0
#                 if self.current_v[i] > 0.0:
#                     self.current_v[i] = max(0.0, self.current_v[i] - step)
#                 # Si la vitesse est négative, on l'augmente vers 0
#                 elif self.current_v[i] < 0.0:
#                     self.current_v[i] = min(0.0, self.current_v[i] + step)
            
#             msg.linear.x = self.current_v[0]
#             msg.linear.y = self.current_v[1]
#             msg.angular.z = self.current_v[2]
            
#         self.cmd_pub.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DockingDebugCLI()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # Sécurité : Forcer l'arrêt physique à la destruction du nœud
#         node.cmd_pub.publish(Twist())
#         node.destroy_node()
#         sys.exit(0)

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import threading
import sys
import math

class OdomPositionController(Node):
    def __init__(self):
        super().__init__('odom_position_controller')
        
        # Configuration
        self.odom_frame = 'odom'
        self.base_frame = 'chassis'
        self.aruco_frame = 'aruco_marker_91'
        
        self.v_max = 0.35
        self.w_max = 0.50
        
        # Gains proportionnels
        self.kp_pos = 1.2
        self.kp_yaw = 1.5
        
        # État cible absolu (dans le repère ODOM)
        self.target_odom = None # [X, Y, Theta]
        self.lock = threading.Lock()
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_docking', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.05, self.control_loop) # 20 Hz
        
        self.get_logger().info("Odom Position Controller Actif.")
        self.get_logger().info("Tapez 'dx dy dyaw' (ex: 0.5 0.0 0.0) ou 'aruco' pour capturer la cible.")
        
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def normalize_angle(self, angle):
        """Maintient l'angle entre -Pi et Pi"""
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def get_transform(self, parent, child):
        """Récupère et extrait [x, y, yaw] d'une TF"""
        try:
            trans = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return [x, y, yaw]
        except TransformException:
            return None

    def set_target(self, dx, dy, dyaw):
        """Génère la cible absolue dans ODOM à partir d'un offset relatif"""
        current_pose = self.get_transform(self.odom_frame, self.base_frame)
        if current_pose is None:
            self.get_logger().error("Impossible de lire l'odométrie (/odom -> /chassis).")
            return

        x0, y0, theta0 = current_pose
        
        # Matrice de rotation 2D pour projeter l'offset local dans le monde absolu
        target_x = x0 + (dx * math.cos(theta0) - dy * math.sin(theta0))
        target_y = y0 + (dx * math.sin(theta0) + dy * math.cos(theta0))
        target_theta = self.normalize_angle(theta0 + dyaw)
        
        with self.lock:
            self.target_odom = [target_x, target_y, target_theta]
            
        self.get_logger().info(f"Nouveau Target ODOM fixé : X={target_x:.2f}, Y={target_y:.2f}, Yaw={target_theta:.2f}")

    def input_loop(self):
        while rclpy.ok():
            try:
                line = input("\n[Consigne] > ").strip()
                if not line: continue
                
                if line.lower() == 'aruco':
                    rel_pose = self.get_transform(self.base_frame, self.aruco_frame)
                    if rel_pose:
                        # On vise 25cm DEVANT le tag
                        dx = rel_pose[0] - 0.25 
                        dy = rel_pose[1]
                        dyaw = rel_pose[2]
                        self.set_target(dx, dy, dyaw)
                    else:
                        print("Tag ArUco non visible ou TF brisée.")
                else:
                    parts = line.split()
                    if len(parts) == 3:
                        self.set_target(float(parts[0]), float(parts[1]), float(parts[2]))
                    else:
                        print("Format invalide. Tapez 'aruco' ou 'dx dy dyaw'.")
            except Exception as e:
                print(f"Erreur d'entrée : {e}")

    def control_loop(self):
        with self.lock:
            if self.target_odom is None:
                return
            target_x, target_y, target_theta = self.target_odom

        current_pose = self.get_transform(self.odom_frame, self.base_frame)
        if current_pose is None:
            self.cmd_pub.publish(Twist()) # Arrêt d'urgence si on perd l'odométrie
            return
            
        x, y, theta = current_pose

        # 1. Calcul de l'erreur dans le repère absolu ODOM
        err_x_odom = target_x - x
        err_y_odom = target_y - y
        err_theta = self.normalize_angle(target_theta - theta)
        
        distance = math.hypot(err_x_odom, err_y_odom)

        # 2. Condition d'arrêt (Tolérance de 2 cm et 1 degré)
        if distance < 0.02 and abs(err_theta) < 0.02:
            self.get_logger().info("Cible atteinte avec succès.")
            self.cmd_pub.publish(Twist())
            with self.lock:
                self.target_odom = None
            return

        # 3. Rotation de l'erreur globale vers le repère local du robot
        # Pour que le robot sache s'il doit avancer (Vx) ou glisser (Vy)
        err_x_local = err_x_odom * math.cos(-theta) - err_y_odom * math.sin(-theta)
        err_y_local = err_x_odom * math.sin(-theta) + err_y_odom * math.cos(-theta)

        # 4. Commandes Proportionnelles
        msg = Twist()
        msg.linear.x = max(min(self.kp_pos * err_x_local, self.v_max), -self.v_max)
        msg.linear.y = max(min(self.kp_pos * err_y_local, self.v_max), -self.v_max)
        msg.angular.z = max(min(self.kp_yaw * err_theta, self.w_max), -self.w_max)
        
        self.cmd_pub.publish(msg)

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
        sys.exit(0)

if __name__ == '__main__':
    main()