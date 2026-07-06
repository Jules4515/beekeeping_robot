#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import NavSatFix, Imu
import math
import os
import threading
from datetime import datetime

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        # Paramètres ROS 2 dynamiques (valeurs par défaut : 2.0m et 15.0°)
        self.declare_parameter('delta_d', 2.0)
        self.declare_parameter('delta_theta', 15.0)
        
        self.delta_d = self.get_parameter('delta_d').value
        self.delta_theta = self.get_parameter('delta_theta').value

        # Thread-safe storage for asynchronous sensor callbacks
        self.gps_lock = threading.Lock()
        self.latest_fix = None
        self.latest_imu = None
        
        # État interne de la mémoire RAM
        self.last_saved_lat = None
        self.last_saved_lon = None
        self.last_saved_alt = None
        self.last_saved_roll = None
        self.last_saved_pitch = None
        self.last_saved_yaw = None
        self.waypoint_counter = 1

        # Configuration du fichier de sortie
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"trajectory_{timestamp}.yaml"
        
        # Pointe vers l'espace source du workspace pour un accès hors-compilation
        ws_path = os.path.expanduser('~/dev/robot_ws/src/bee_mobile/trajectories')
        os.makedirs(ws_path, exist_ok=True)
        self.yaml_path = os.path.join(ws_path, filename)

        # Initialisation du fichier YAML
        with open(self.yaml_path, 'w') as f:
            f.write("waypoints_GPS:\n\n")

        # Subscriptions
        self.fix_sub = self.create_subscription(NavSatFix, '/fix', self.fix_callback, 10)
        self.heading_sub = self.create_subscription(Imu, '/heading_imu', self.imu_callback, 10)

        # Boucle de traitement à 20 Hz pour évaluer la trajectoire en continu
        self.timer = self.create_timer(0.05, self.process_trajectory)
        
        self.get_logger().info("=== SPATIAL TRAJECTORY RECORDER INITIALIZED ===")
        self.get_logger().info(f"Paramètres : Delta D = {self.delta_d}m | Delta Theta = {self.delta_theta}°")
        self.get_logger().info(f"Cible      : {self.yaml_path}")

    def fix_callback(self, msg):
        with self.gps_lock:
            self.latest_fix = msg
    
    def imu_callback(self, msg):
        with self.gps_lock:
            self.latest_imu = msg

    @staticmethod
    def euler_from_quaternion(q):
        """Converts quaternion to euler angles (ZYX order) to extract physical yaw."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw)

    @staticmethod
    def calculate_haversine_distance(lat1, lon1, lat2, lon2):
        """Calcule la distance absolue en mètres entre deux points GPS."""
        R = 6371000.0  # Rayon terrestre en mètres
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        
        a = math.sin(dphi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0)**2
        return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))

    @staticmethod
    def calculate_angular_difference(yaw1, yaw2):
        """Calcule la différence d'angle minimale en gérant le franchissement -180/180."""
        diff = yaw1 - yaw2
        return (diff + 180) % 360 - 180

    def process_trajectory(self):
        """Logique de filtrage spatial exécutée à haute fréquence."""
        with self.gps_lock:
            if not self.latest_fix or not self.latest_imu:
                return
            
            current_lat = self.latest_fix.latitude
            current_lon = self.latest_fix.longitude
            current_alt = self.latest_fix.altitude
            
            # Extraction complète de l'attitude 6DoF
            current_roll, current_pitch, current_yaw = self.euler_from_quaternion(self.latest_imu.orientation)

        # 1. Sauvegarde inconditionnelle du premier point (Origine)
        if self.last_saved_lat is None:
            self.save_to_disk(current_lat, current_lon, current_alt, current_roll, current_pitch, current_yaw)
            return

        # 2. Calcul des deltas physiques (Reste inchangé)
        dist = self.calculate_haversine_distance(self.last_saved_lat, self.last_saved_lon, current_lat, current_lon)
        angle_diff = abs(self.calculate_angular_difference(self.last_saved_yaw, current_yaw))

        # 3. Évaluation conditionnelle (Filtre Spatial)
        if dist >= self.delta_d or angle_diff >= self.delta_theta:
            self.save_to_disk(current_lat, current_lon, current_alt, current_roll, current_pitch, current_yaw)

    def save_to_disk(self, lat, lon, alt, roll, pitch, yaw):
        """Écriture asynchrone sur le disque de la Pose 6DoF."""
        try:
            with open(self.yaml_path, 'a') as f:
                f.write(f"- name      : WP_{self.waypoint_counter}\n")
                f.write(f"  latitude  : {lat:.7f}\n")
                f.write(f"  longitude : {lon:.7f}\n")
                f.write(f"  altitude  : {alt:.2f}\n")
                f.write(f"  roll      : {roll:.2f}\n")
                f.write(f"  pitch     : {pitch:.2f}\n")
                f.write(f"  yaw       : {yaw:.2f}\n\n")

            # Mise à jour du cache RAM
            self.last_saved_lat = lat
            self.last_saved_lon = lon
            self.last_saved_alt = alt
            self.last_saved_roll = roll
            self.last_saved_pitch = pitch
            self.last_saved_yaw = yaw
            
            self.get_logger().info(f"[WP_{self.waypoint_counter}] Enregistré | Dist: {self.delta_d}m ou Cap: {self.delta_theta}° franchi.")
            self.waypoint_counter += 1

        except IOError as e:
            self.get_logger().error(f"YAML Write Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("\nArrêt de l'enregistrement. Trajectoire finale sauvegardée.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()