#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class CompressedArucoNode(Node):
    def __init__(self):
        super().__init__('compressed_aruco_node')
        
        # Déclaration des paramètres
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('marker_size', 0.068)
        
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

        # Paramètres de détection OpenCV optimisés
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # Variables pour la calibration (Matrice K et Distorsion D)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Écoute unique des infos de calibration de la caméra
        self.info_sub = self.create_subscription(
            CameraInfo,
            self.info_topic,
            self.camera_info_callback,
            10
        )

        # Publishers
        self.marker_pub = self.create_publisher(ArucoMarkers, '/aruco_markers', 10)
        self.image_pub = self.create_publisher(Image, '~/debug_image', 10)

        # Souscription au flux compressé
        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.compressed_image_callback,
            10
        )
        
        self.get_logger().info(f"⚡ Nœud ArUco Pose 3D connecté sur : {self.image_topic}")

    def camera_info_callback(self, msg):
        """ Récupère les données de calibration une seule fois """
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("✅ Paramètres de calibration caméra reçus et chargés.")
        # On détruit la souscription pour libérer des ressources car la calibration ne change pas
        self.destroy_subscription(self.info_sub)

    def compressed_image_callback(self, msg):
        # On ne calcule pas la pose 3D si on n'a pas encore reçu la calibration
        if self.camera_matrix is None:
            return

        try:
            # Décodage de l'image compressée
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)

            # Détection des marqueurs
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
            
            if ids is not None and len(ids) > 0:
                # Préparation du message ROS2 personnalisé
                markers_msg = ArucoMarkers()
                markers_msg.header.stamp = msg.header.stamp
                markers_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else "camera_link_optical"

                # Estimation des vecteurs de Rotation (rvecs) et Translation (tvecs)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                )

                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    markers_msg.marker_ids.append(marker_id)

                    # Dessiner les axes 3D sur l'image de debug
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)

                    # Instanciation de la Pose
                    pose = Pose()
                    
                    # Position (Translation vectorielle)
                    pose.position.x = float(tvecs[i][0][0])
                    pose.position.y = float(tvecs[i][0][1])
                    pose.position.z = float(tvecs[i][0][2])

                    # Orientation : Conversion du vecteur de rotation d'OpenCV (Rodrigues) en Quaternion pour ROS
                    r_matrix, _ = cv2.Rodrigues(rvecs[i])
                    
                    # Algorithme d'extraction de Quaternion depuis une matrice de rotation
                    t = np.trace(r_matrix)
                    if t > 0:
                        M = np.sqrt(t + 1.0) * 2
                        pose.orientation.w = 0.25 * M
                        pose.orientation.x = (r_matrix[2, 1] - r_matrix[1, 2]) / M
                        pose.orientation.y = (r_matrix[0, 2] - r_matrix[2, 0]) / M
                        pose.orientation.z = (r_matrix[1, 0] - r_matrix[0, 1]) / M
                    else:
                        if (r_matrix[0, 0] > r_matrix[1, 1]) and (r_matrix[0, 0] > r_matrix[2, 2]):
                            M = np.sqrt(1.0 + r_matrix[0, 0] - r_matrix[1, 1] - r_matrix[2, 2]) * 2
                            pose.orientation.w = (r_matrix[2, 1] - r_matrix[1, 2]) / M
                            pose.orientation.x = 0.25 * M
                            pose.orientation.y = (r_matrix[0, 1] + r_matrix[1, 0]) / M
                            pose.orientation.z = (r_matrix[0, 2] + r_matrix[2, 0]) / M
                        elif r_matrix[1, 1] > r_matrix[2, 2]:
                            M = np.sqrt(1.0 + r_matrix[1, 1] - r_matrix[0, 0] - r_matrix[2, 2]) * 2
                            pose.orientation.w = (r_matrix[0, 2] - r_matrix[2, 0]) / M
                            pose.orientation.x = (r_matrix[0, 1] + r_matrix[1, 0]) / M
                            pose.orientation.y = 0.25 * M
                            pose.orientation.z = (r_matrix[1, 2] + r_matrix[2, 1]) / M
                        else:
                            M = np.sqrt(1.0 + r_matrix[2, 2] - r_matrix[0, 0] - r_matrix[1, 1]) * 2
                            pose.orientation.w = (r_matrix[1, 0] - r_matrix[0, 1]) / M
                            pose.orientation.x = (r_matrix[0, 2] + r_matrix[2, 0]) / M
                            pose.orientation.y = (r_matrix[1, 2] + r_matrix[2, 1]) / M
                            pose.orientation.z = 0.25 * M

                    markers_msg.poses.append(pose)

                # Publication des coordonnées 3D
                self.marker_pub.publish(markers_msg)

                # Publication de la vidéo de débogage (avec repères 3D dessinés sur le tag)
                debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(debug_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"Erreur : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CompressedArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()