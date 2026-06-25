#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class TfMathTester(Node):
    def __init__(self):
        super().__init__('tf_math_tester')
        
        # Frames à observer
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        self.aruco_frame = 'aruco_marker_91'
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Boucle de test très lente (2 Hz) pour laisser le temps de lire le terminal
        self.timer = self.create_timer(0.5, self.diagnostic_loop)
        
        self.get_logger().info("TF MATH TESTER [ACTIF] - En attente des transformations...")

    def normalize_angle(self, angle):
        """Maintient l'angle strictement entre -PI et +PI."""
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def get_transform(self, parent, child):
        """Test de l'extraction de l'odométrie 2D (Yaw)."""
        try:
            trans = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.05))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            
            # Conversion Quaternion -> Yaw (Rotation autour de Z)
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return [x, y, yaw]
        except TransformException:
            return None

    def get_tag_quaternion(self, parent, child):
        """Test de l'extraction 3D brute du Tag ArUco."""
        try:
            trans = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.05))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            q = trans.transform.rotation
            return [x, y, z, q.x, q.y, q.z, q.w]
        except TransformException:
            return None

    def diagnostic_loop(self):
        # 1. Mesure de l'Odométrie 2D
        pose_robot = self.get_transform(self.odom_frame, self.base_frame)
        if not pose_robot:
            return # Attend silencieusement que l'odométrie soit disponible
            
        rx, ry, ryaw = pose_robot

        # 2. Mesure du Tag 3D dans le repère du robot
        pose_tag = self.get_tag_quaternion(self.base_frame, self.aruco_frame)
        
        if pose_tag:
            tx, ty, tz, qx, qy, qz, qw = pose_tag
            
            # --- MATHÉMATIQUES DU VECTEUR NORMAL ---
            # Extraction des composantes X et Y du vecteur Z (sortant) du tag
            vec_z_x = 2.0 * (qx * qz + qw * qy)
            vec_z_y = 2.0 * (qy * qz - qw * qx)
            
            # Angle du vecteur normal de la ruche par rapport au châssis du robot
            tag_normal_angle = math.atan2(vec_z_y, vec_z_x)
            
            # Angle géométrique cible (anti-parallèle)
            dtheta_target = self.normalize_angle(tag_normal_angle + math.pi)
            
            # Affichage de diagnostic
            self.get_logger().info(
                f"\n=== DIAGNOSTIC GÉOMÉTRIQUE (2 Hz) ===\n"
                f"[ROBOT DANS ODOM]\n"
                f"  X: {rx:>6.3f}m | Y: {ry:>6.3f}m | Yaw: {math.degrees(ryaw):>6.1f}°\n"
                f"[TAG 3D BRUT DANS BASE_LINK]\n"
                f"  X: {tx:>6.3f}m | Y: {ty:>6.3f}m | Z (Hauteur): {tz:>6.3f}m\n"
                f"  Qx: {qx:>5.2f} | Qy: {qy:>5.2f} | Qz: {qz:>5.2f} | Qw: {qw:>5.2f}\n"
                f"[EXTRACTION NORMALE (PROJECTION 2D)]\n"
                f"  Vecteur Normal Ruche : X_proj={vec_z_x:>5.2f}, Y_proj={vec_z_y:>5.2f}\n"
                f"  Angle Normale Ruche  : {math.degrees(tag_normal_angle):>6.1f}°\n"
                f"[ERREUR D'ALIGNEMENT ROBOT]\n"
                f"  Correction Wz requise: {math.degrees(dtheta_target):>6.1f}°\n"
                f"====================================="
            )
        else:
            # Affichage alternatif si le tag n'est pas vu
            self.get_logger().info(
                f"\n=== DIAGNOSTIC GÉOMÉTRIQUE (2 Hz) ===\n"
                f"[ROBOT DANS ODOM]\n"
                f"  X: {rx:>6.3f}m | Y: {ry:>6.3f}m | Yaw: {math.degrees(ryaw):>6.1f}°\n"
                f"[TAG ARUCO]\n"
                f"  NON DÉTECTÉ / HORS CHAMP\n"
                f"====================================="
            )

def main(args=None):
    rclpy.init(args=args)
    node = TfMathTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()