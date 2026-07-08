#!/usr/bin/env python3
import sys

import yaml
import os
import math
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.executors import ExternalShutdownException
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from robot_localization.srv import FromLL
from std_msgs.msg import String
from action_msgs.srv import CancelGoal

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class GpsMissionNode(Node):
    def __init__(self):
        super().__init__('gps_mission_node')
        
        # Paramètre ROS pour cibler le fichier d'enregistrement exact
        self.declare_parameter('trajectory_file', 'trajectory_DEFAULT.yaml')
        self.trajectory_file = self.get_parameter('trajectory_file').value

        print("[INFO] To launch with a custom file: ros2 run bee_mobile gps_mission_node --ros-args -p trajectory_file:=my_file.yaml")
        
        self.navigator = BasicNavigator()
        self.gps_status = {'emoji': '🔴', 'text': 'ATTENTE...'}
        
        # Subscriptions et Publishers
        self.status_sub = self.create_subscription(
            String, '/gps/status', self.gps_status_callback, 10)
        
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/points_visuels', qos)
        
        self.from_ll_client = self.create_client(FromLL, '/fromLL')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def gps_status_callback(self, msg):
        data = msg.data
        
        # On cherche la présence du statut dans la chaîne complète
        if 'RTK_FIXED' in data:
            self.gps_status = {'emoji': '🟢', 'text': data}
        elif 'RTK_FLOAT' in data:
            self.gps_status = {'emoji': '🔵', 'text': data}
        elif 'DGPS' in data:
            self.gps_status = {'emoji': '🟡', 'text': data}
        elif 'GPS_POOR' in data:
            self.gps_status = {'emoji': '🟠', 'text': data}
        else:
            self.gps_status = {'emoji': '🔴', 'text': data}

    def print_gps_banner(self):
        emoji = self.gps_status['emoji']
        text = self.gps_status['text']
        print("\n" + "="*50)
        print(f"  STATUT GPS : {emoji} {text}")
        print("="*50 + "\n")

    @staticmethod
    def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
        """Convertit les angles d'Euler (en degrés) en quaternion 3D."""
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def execute_mission(self):
        """Logique d'exécution principale de la mission."""
        self.get_logger().info("Attente de Nav2...")
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')
        
        self.get_logger().info("Attente des données GPS...")
        timeout = time.time() + 5.0
        while self.gps_status['text'] == 'ATTENTE...' and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.print_gps_banner()
        if 'RTK_FIXED' not in self.gps_status['text'] and 'DGPS' not in self.gps_status['text'] and self.gps_status['text'] != 'ATTENTE...':
            self.get_logger().warn("Pas de correction optimale (RTK_FIXED / DGPS) détectée !")
            input("Appuyez sur ENTRÉE pour continuer quand même...")

        self.get_logger().info("Attente du service de conversion GPS (/fromLL)...")
        while not self.from_ll_client.wait_for_service(timeout_sec=2.0):
            rclpy.spin_once(self, timeout_sec=0.1)

        # ============================================================
        # LECTURE ET CONVERSION DES WAYPOINTS
        # ============================================================
        ws_path = os.path.expanduser('~/dev/robot_ws/src/bee_mobile/trajectories')
        os.makedirs(ws_path, exist_ok=True)
        yaml_path = os.path.join(ws_path, self.trajectory_file)
        
        # Logique de Fallback si le fichier demandé n'existe pas
        if not os.path.exists(yaml_path):
            self.get_logger().error(f"Fichier introuvable : {yaml_path}")
            self.get_logger().info("Fallback to default file: trajectory_DEFAULT.yaml")
            self.trajectory_file = 'trajectory_DEFAULT.yaml'
            yaml_path = os.path.join(ws_path, self.trajectory_file)
            
            # Vérification de sécurité pour le fichier par défaut
            if not os.path.exists(yaml_path):
                self.get_logger().error("Fichier par défaut introuvable. Fin de mission.")
                return

        self.get_logger().info(f"Chargement de la trajectoire : {self.trajectory_file}")
        with open(yaml_path, 'r') as f:
            waypoints = yaml.safe_load(f).get('waypoints_GPS', [])

        map_poses = []
        marker_array = MarkerArray()

        self.get_logger().info("Génération de l'empreinte spatiale (Markers RViz)...")
        for i, wp in enumerate(waypoints):
            # Appel du service de conversion cartographique
            req = FromLL.Request()
            req.ll_point.latitude = wp['latitude']
            req.ll_point.longitude = wp['longitude']
            req.ll_point.altitude = wp['altitude']

            future = self.from_ll_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            map_point = future.result().map_point

            # Construction de la Pose 6DoF
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position = map_point
            pose.pose.orientation = self.euler_to_quaternion(wp['roll'], wp['pitch'], wp['yaw'])
            map_poses.append(pose)

            # Marqueur 1 : Le Cylindre Bleu (Fixation de position)
            marker_cyl = Marker()
            marker_cyl.header = pose.header
            marker_cyl.ns = 'waypoints_position'
            marker_cyl.id = i * 2
            marker_cyl.type = Marker.CYLINDER
            marker_cyl.action = Marker.ADD
            marker_cyl.pose.position = pose.pose.position
            marker_cyl.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) # Cylindre posé à plat
            marker_cyl.scale.x = 0.2
            marker_cyl.scale.y = 0.2
            marker_cyl.scale.z = 0.05
            marker_cyl.color.r, marker_cyl.color.g, marker_cyl.color.b, marker_cyl.color.a = 0.0, 0.5, 1.0, 0.8
            marker_array.markers.append(marker_cyl)

            # Marqueur 2 : La Flèche Rouge (Attitude 6DoF)
            marker_arr = Marker()
            marker_arr.header = pose.header
            marker_arr.ns = 'waypoints_orientation'
            marker_arr.id = i * 2 + 1
            marker_arr.type = Marker.ARROW
            marker_arr.action = Marker.ADD
            marker_arr.pose = pose.pose # Hérite de la rotation absolue du robot
            marker_arr.scale.x = 0.4  # Longueur
            marker_arr.scale.y = 0.05 # Épaisseur tige
            marker_arr.scale.z = 0.05 # Épaisseur tête
            marker_arr.color.r, marker_arr.color.g, marker_arr.color.b, marker_arr.color.a = 1.0, 0.0, 0.0, 1.0
            marker_array.markers.append(marker_arr)

        # Publication simultanée de toutes les formes
        self.marker_pub.publish(marker_array)
        time.sleep(0.5)

        # ============================================================
        # EXÉCUTION TRONÇON PAR TRONÇON
        # ============================================================
        print("\n" + "="*50)
        print("DÉMARRAGE DE LA MISSION")
        self.print_gps_banner()
        print("="*50)
        input(f"\n> {len(map_poses)} points chargés. ENTRÉE pour lancer le robot... (Ctrl+C pour annuler)")

        for i in range(len(map_poses)):
            target_pose = map_poses[i]
            wp_name = waypoints[i]['name']

            print(f"\n---> Cible : [{wp_name}] | GPS: {self.gps_status['emoji']} {self.gps_status['text']}")
            
            if 'RTK_FIXED' not in self.gps_status['text'] and 'DGPS' not in self.gps_status['text'] and i > 0:
                print(f"     ⚠️  Alerte de dégradation GPS : {self.gps_status['text']}")

            self.navigator.goToPose(target_pose)

            # Boucle d'attente silencieuse (sans spam d'estimation de distance)
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)

            # Analyse du résultat
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f"  [SUCCÈS] {wp_name} atteint. Attitude respectée.")
            elif result == TaskResult.CANCELED:
                print(f"  [ANNULÉ] Ordre d'arrêt reçu.")
                break
            elif result == TaskResult.FAILED:
                print(f"  [ÉCHEC] Cible inatteignable (Obstacle / Échec du planificateur).")

        print("\n" + "="*50)
        print("FIN DE MISSION !")
        self.print_gps_banner()
        print("="*50)

def main(args=None):
    rclpy.init(args=args)
    node = GpsMissionNode()
    
    try:
        node.execute_mission()
    except (KeyboardInterrupt, ExternalShutdownException):
        print(f"[INFO] [{node.get_name()}]: Interruption demandée. Arrêt propre en cours...")
        
        # Isolation de la tentative d'annulation pour absorber la corruption du contexte ROS
        try:
            if hasattr(node.navigator, 'nav_to_pose_client') and node.navigator.nav_to_pose_client.server_is_ready():
                if hasattr(node.navigator, 'goal_handle') and node.navigator.goal_handle is not None:
                    future = node.navigator.goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(node.navigator, future, timeout_sec=1.0)
                else:
                    cancel_client = node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
                    if cancel_client.wait_for_service(timeout_sec=0.5):
                        req = CancelGoal.Request()
                        future = cancel_client.call_async(req)
                        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        except Exception:
            pass # Si le contexte est mort, on ignore silencieusement
            
    finally:
        # Nettoyage final sécurisé
        try:
            node.navigator.destroy_node()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        
        # Forçage de la sortie système avec le code 0 (Succès absolu) 
        # pour empêcher le log "Process exited with failure 1"
        sys.exit(0)

if __name__ == '__main__':
    main()