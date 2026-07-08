#!/usr/bin/env python3
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
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from robot_localization.srv import FromLL
from std_msgs.msg import String
from action_msgs.srv import CancelGoal

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory

class GpsMissionNode(Node):
    def __init__(self):
        super().__init__('gps_mission_node')
        
        # 1. Encapsulation du navigateur Nav2
        self.navigator = BasicNavigator()
        
        # 2. Gestion de l'état interne (Remplace la variable globale)
        self.gps_status = {'emoji': '🔴', 'text': 'ATTENTE...'}
        
        # 3. Subscriptions et Publishers
        self.status_sub = self.create_subscription(
            String, '/gps/status', self.gps_status_callback, 10)
        
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/chemin_previsualisation', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/points_visuels', qos)
        
        # 4. Service Client
        self.from_ll_client = self.create_client(FromLL, '/fromLL')
        
        # 5. TF2 (Écouteur de transformations)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def gps_status_callback(self, msg):
        data = msg.data
        if data == 'RTK_FIXED':
            self.gps_status = {'emoji': '🟢', 'text': 'RTK FIXED'}
        elif data == 'DGPS':
            self.gps_status = {'emoji': '🟡', 'text': 'DGPS'}
        elif data == 'GPS':
            self.gps_status = {'emoji': '🟠', 'text': 'GPS'}
        else:
            self.gps_status = {'emoji': '🔴', 'text': 'PAS DE FIX'}

    def print_gps_banner(self):
        emoji = self.gps_status['emoji']
        text = self.gps_status['text']
        print("\n" + "="*50)
        print(f"  STATUT GPS : {emoji} {text}")
        print("="*50 + "\n")

    @staticmethod
    def get_quaternion_from_yaw(yaw_deg):
        yaw_rad = math.radians(yaw_deg)
        q = Quaternion()
        q.w = math.cos(yaw_rad / 2.0)
        q.z = math.sin(yaw_rad / 2.0)
        return q

    def get_current_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = transform.transform.rotation
            return pose
        except Exception:
            return None

    def execute_mission(self):
        """Logique d'exécution principale de la mission."""
        self.get_logger().info("Attente de Nav2...")
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')
        
        self.get_logger().info("Attente des premières données GPS...")
        timeout = time.time() + 5.0
        while self.gps_status['text'] == 'ATTENTE...' and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.print_gps_banner()
        
        if self.gps_status['text'] not in ['RTK FIXED', 'DGPS'] and self.gps_status['text'] != 'ATTENTE...':
            self.get_logger().warn("Pas de correction RTK/DGPS détectée !")
            input("Appuyez sur ENTRÉE pour continuer quand même...")

        self.get_logger().info("Attente du service de conversion GPS (/fromLL)...")
        while not self.from_ll_client.wait_for_service(timeout_sec=2.0):
            rclpy.spin_once(self, timeout_sec=0.1)

        # ============================================================
        # LECTURE ET CONVERSION DES WAYPOINTS
        # ============================================================
        pkg_share = get_package_share_directory('bee_mobile') # Vérifie que ce nom correspond bien à ton package
        yaml_path = os.path.join(pkg_share, 'config', 'waypoints_GPS.yaml')
        
        with open(yaml_path, 'r') as f:
            waypoints = yaml.safe_load(f).get('waypoints_GPS', [])

        map_poses = []
        marker_array = MarkerArray()

        self.get_logger().info("Conversion des waypoints GPS en coordonnées map...")
        for i, wp in enumerate(waypoints):
            req = FromLL.Request()
            req.ll_point.latitude = wp['latitude']
            req.ll_point.longitude = wp['longitude']
            req.ll_point.altitude = 0.0

            future = self.from_ll_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            map_point = future.result().map_point

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position = map_point
            pose.pose.orientation = self.get_quaternion_from_yaw(wp['yaw'])
            map_poses.append(pose)

            marker = Marker()
            marker.header = pose.header
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = 0.1
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.9
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        time.sleep(1.0) # Laisser le temps à RViz d'afficher les marqueurs

        # ============================================================
        # EXÉCUTION TRONÇON PAR TRONÇON
        # ============================================================
        print("\n" + "="*50)
        print("DÉMARRAGE DE LA MISSION (Mode Rolling Window)")
        self.print_gps_banner()
        print("="*50)
        input("\n> ENTRÉE pour lancer le robot... (Ctrl+C pour annuler)")

        for i in range(len(map_poses)):
            target_pose = map_poses[i]
            wp_name = waypoints[i]['name']
            wait_time = waypoints[i].get('wait_time', 0.0)

            print(f"\n---> ÉTAPE {i+1}/{len(map_poses)} : En route vers [{wp_name}]")
            print(f"     GPS: {self.gps_status['emoji']} {self.gps_status['text']}")

            if self.gps_status['text'] not in ['RTK FIXED', 'DGPS'] and i > 0:
                print(f"     ⚠️  Alerte GPS : {self.gps_status['text']}")

            current_pose = self.get_current_pose()
            if current_pose is None:
                current_pose = map_poses[i-1] if i > 0 else target_pose

            path = self.navigator.getPath(current_pose, target_pose, planner_id="GridBased")
            if path is not None:
                self.path_pub.publish(path)
                print("  -> Chemin local généré et publié sur RViz.")
            else:
                print("  -> [ATTENTION] Impossible de générer la prévisualisation. Tentative quand même.")

            self.navigator.goToPose(target_pose)

            j = 0
            while not self.navigator.isTaskComplete():
                j += 1
                # On spin notre Node personnalisé manuellement pour continuer de recevoir /gps/status et les TF
                rclpy.spin_once(self, timeout_sec=0.05)
                
                feedback = self.navigator.getFeedback()
                if feedback and j % 5 == 0:
                    print(f"  Progression : {feedback.distance_remaining:.1f} m restants | GPS: {self.gps_status['emoji']} {self.gps_status['text']}")

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f"  [SUCCÈS] {wp_name} atteint ! Orientation finale respectée.")
                if wait_time > 0:
                    print(f"  Attente de {wait_time}s...")
                    time.sleep(wait_time)
            elif result == TaskResult.CANCELED:
                print(f"  [ANNULÉ] Navigation vers {wp_name} annulée.")
                break
            elif result == TaskResult.FAILED:
                print(f"  [ÉCHEC] Le robot n'a pas pu atteindre {wp_name} (Point abandonné).")

        print("\n" + "="*50)
        print("FIN DE MISSION !")
        self.print_gps_banner()
        print("="*50)

        # A TESTER MAIS MARCHE PAS, ARRETE LA MISSION DIRECT JSP PK
        # # ============================================================
        # # EXÉCUTION FLUIDE DE LA TRAJECTOIRE COMPLETE
        # # ============================================================
        
        # print("\n" + "="*50)
        # print("DÉMARRAGE DE LA MISSION CONTINUE (NavigateThroughPoses)")
        # self.print_gps_banner()
        # print("="*50)
        # input("\n> ENTRÉE pour lancer le robot... (Ctrl+C pour annuler)")

        # # On envoie toute la liste de coordonnées à Nav2 en une seule fois
        # self.navigator.goThroughPoses(map_poses)

        # i = 0
        # while not self.navigator.isTaskComplete():
        #     i += 1
        #     # On spin notre Node personnalisé manuellement pour continuer de recevoir /gps/status
        #     rclpy.spin_once(self, timeout_sec=0.05)
            
        #     feedback = self.navigator.getFeedback()
        #     if feedback and i % 10 == 0:
        #         # Affichage allégé pour ne pas spammer le terminal
        #         # feedback.distance_remaining donne la distance jusqu'au DERNIER point de la liste
        #         print(f"  En route... | {feedback.distance_remaining:.1f} m restants jusqu'à la fin | GPS: {self.gps_status['emoji']} {self.gps_status['text']}")

        # # Analyse du résultat final de la trajectoire
        # result = self.navigator.getResult()
        # if result == TaskResult.SUCCEEDED:
        #     print("\n  [SUCCÈS] Trajectoire complète achevée !")
        # elif result == TaskResult.CANCELED:
        #     print("\n  [ANNULÉ] Mission interrompue par l'utilisateur ou le joystick.")
        # elif result == TaskResult.FAILED:
        #     print("\n  [ÉCHEC] Le robot n'a pas pu terminer la trajectoire complète.")

        # print("\n" + "="*50)
        # print("FIN DE MISSION !")
        # self.print_gps_banner()
        # print("="*50)

def main(args=None):
    rclpy.init(args=args)
    node = GpsMissionNode()
    
    try:
        node.execute_mission()
    except (KeyboardInterrupt, ExternalShutdownException):
        print(f"[INFO] [{node.get_name()}]: Shutdown requested by user. Canceling current goal...")
        
        # 1. Appel natif du cancel via l'ActionClient du Navigator
        # Si tu utilises le BasicNavigator standard de Nav2, il possède un attribut 'nav_to_pose_client'
        if hasattr(node.navigator, 'nav_to_pose_client') and node.navigator.nav_to_pose_client.server_is_ready():
            # Si le navigator possède un goal_handle actif, on l'annule
            if hasattr(node.navigator, 'goal_handle') and node.navigator.goal_handle is not None:
                future = node.navigator.goal_handle.cancel_goal_async()
                
                # On fait tourner brièvement le nœud pour traiter l'envoi du message de cancel
                rclpy.spin_until_future_complete(node.navigator, future, timeout_sec=2.0)
                print(f"[INFO] [{node.get_name()}]: Goal cancellation request sent successfully.")
            else:
                # Alternative brute : Appel direct du service si aucun goal_handle n'est accessible directement
                cancel_client = node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
                if cancel_client.wait_for_service(timeout_sec=1.0):
                    req = CancelGoal.Request()
                    # Une requête vide "{}" cible le goal actif ou tous les goals selon le protocole
                    future = cancel_client.call_async(req)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                    print(f"[INFO] [{node.get_name()}]: Generic cancel service called.")
        
    finally:
        # Destruction propre des deux noeuds (Notre classe + BasicNavigator interne)
        node.navigator.destroy_node()
        node.destroy_node()
        
        # Vérification du contexte pour éviter rclpy._rclpy_pybind11.RCLError
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()