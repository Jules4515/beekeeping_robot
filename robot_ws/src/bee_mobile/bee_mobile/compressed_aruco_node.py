# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage, Image, CameraInfo
# from geometry_msgs.msg import Pose
# from ros2_aruco_interfaces.msg import ArucoMarkers
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np

# class CompressedArucoNode(Node):
#     def __init__(self):
#         super().__init__('compressed_aruco_node')
        
#         # Déclaration des paramètres
#         self.declare_parameter('image_topic', '/camera/image_raw/compressed')
#         self.declare_parameter('camera_info_topic', '/camera/camera_info')
#         self.declare_parameter('marker_size', 0.068)
        
#         self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
#         self.info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
#         self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value

#         self.bridge = CvBridge()
#         self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

#         # Paramètres de détection OpenCV optimisés
#         self.parameters = cv2.aruco.DetectorParameters_create()
#         self.parameters.adaptiveThreshWinSizeMin = 3
#         self.parameters.adaptiveThreshWinSizeMax = 23
#         self.parameters.adaptiveThreshWinSizeStep = 10
#         self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

#         # Variables pour la calibration (Matrice K et Distorsion D)
#         self.camera_matrix = None
#         self.dist_coeffs = None
        
#         # Écoute unique des infos de calibration de la caméra
#         self.info_sub = self.create_subscription(
#             CameraInfo,
#             self.info_topic,
#             self.camera_info_callback,
#             10
#         )

#         # Publishers
#         self.marker_pub = self.create_publisher(ArucoMarkers, '/aruco_markers', 10)
#         self.image_pub = self.create_publisher(Image, '~/debug_image', 10)

#         # Souscription au flux compressé
#         self.image_sub = self.create_subscription(
#             CompressedImage,
#             self.image_topic,
#             self.compressed_image_callback,
#             10
#         )
        
#         self.get_logger().info(f"⚡ Nœud ArUco Pose 3D connecté sur : {self.image_topic}")

#     def camera_info_callback(self, msg):
#         """ Récupère les données de calibration une seule fois """
#         self.camera_matrix = np.array(msg.k).reshape((3, 3))
#         self.dist_coeffs = np.array(msg.d)
#         self.get_logger().info("✅ Paramètres de calibration caméra reçus et chargés.")
#         # On détruit la souscription pour libérer des ressources car la calibration ne change pas
#         self.destroy_subscription(self.info_sub)

#     def compressed_image_callback(self, msg):
#         # On ne calcule pas la pose 3D si on n'a pas encore reçu la calibration
#         if self.camera_matrix is None:
#             return

#         try:
#             # Décodage de l'image compressée
#             frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#             gray = cv2.equalizeHist(gray)

#             # Détection des marqueurs
#             corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
            
#             if ids is not None and len(ids) > 0:
#                 # Préparation du message ROS2 personnalisé
#                 markers_msg = ArucoMarkers()
#                 markers_msg.header.stamp = msg.header.stamp
#                 markers_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else "camera_link_optical"

#                 # Estimation des vecteurs de Rotation (rvecs) et Translation (tvecs)
#                 rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
#                     corners, self.marker_size, self.camera_matrix, self.dist_coeffs
#                 )

#                 cv2.aruco.drawDetectedMarkers(frame, corners, ids)

#                 for i in range(len(ids)):
#                     marker_id = int(ids[i][0])
#                     markers_msg.marker_ids.append(marker_id)

#                     # Dessiner les axes 3D sur l'image de debug
#                     cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)

#                     # Instanciation de la Pose
#                     pose = Pose()
                    
#                     # Position (Translation vectorielle)
#                     pose.position.x = float(tvecs[i][0][0])
#                     pose.position.y = float(tvecs[i][0][1])
#                     pose.position.z = float(tvecs[i][0][2])

#                     # Orientation : Conversion du vecteur de rotation d'OpenCV (Rodrigues) en Quaternion pour ROS
#                     r_matrix, _ = cv2.Rodrigues(rvecs[i])
                    
#                     # Algorithme d'extraction de Quaternion depuis une matrice de rotation
#                     t = np.trace(r_matrix)
#                     if t > 0:
#                         M = np.sqrt(t + 1.0) * 2
#                         pose.orientation.w = 0.25 * M
#                         pose.orientation.x = (r_matrix[2, 1] - r_matrix[1, 2]) / M
#                         pose.orientation.y = (r_matrix[0, 2] - r_matrix[2, 0]) / M
#                         pose.orientation.z = (r_matrix[1, 0] - r_matrix[0, 1]) / M
#                     else:
#                         if (r_matrix[0, 0] > r_matrix[1, 1]) and (r_matrix[0, 0] > r_matrix[2, 2]):
#                             M = np.sqrt(1.0 + r_matrix[0, 0] - r_matrix[1, 1] - r_matrix[2, 2]) * 2
#                             pose.orientation.w = (r_matrix[2, 1] - r_matrix[1, 2]) / M
#                             pose.orientation.x = 0.25 * M
#                             pose.orientation.y = (r_matrix[0, 1] + r_matrix[1, 0]) / M
#                             pose.orientation.z = (r_matrix[0, 2] + r_matrix[2, 0]) / M
#                         elif r_matrix[1, 1] > r_matrix[2, 2]:
#                             M = np.sqrt(1.0 + r_matrix[1, 1] - r_matrix[0, 0] - r_matrix[2, 2]) * 2
#                             pose.orientation.w = (r_matrix[0, 2] - r_matrix[2, 0]) / M
#                             pose.orientation.x = (r_matrix[0, 1] + r_matrix[1, 0]) / M
#                             pose.orientation.y = 0.25 * M
#                             pose.orientation.z = (r_matrix[1, 2] + r_matrix[2, 1]) / M
#                         else:
#                             M = np.sqrt(1.0 + r_matrix[2, 2] - r_matrix[0, 0] - r_matrix[1, 1]) * 2
#                             pose.orientation.w = (r_matrix[1, 0] - r_matrix[0, 1]) / M
#                             pose.orientation.x = (r_matrix[0, 2] + r_matrix[2, 0]) / M
#                             pose.orientation.y = (r_matrix[1, 2] + r_matrix[2, 1]) / M
#                             pose.orientation.z = 0.25 * M

#                     markers_msg.poses.append(pose)

#                 # Publication des coordonnées 3D
#                 self.marker_pub.publish(markers_msg)

#                 # Publication de la vidéo de débogage (avec repères 3D dessinés sur le tag)
#                 debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
#                 self.image_pub.publish(debug_msg)

#         except CvBridgeError as e:
#             self.get_logger().error(f"Erreur : {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = CompressedArucoNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage, Image, CameraInfo
# from geometry_msgs.msg import Pose
# from ros2_aruco_interfaces.msg import ArucoMarkers
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np
# import time # Importé pour les mesures de précision

# class CompressedArucoNode(Node):
#     def __init__(self):
#         super().__init__('compressed_aruco_node')
        
#         # Déclaration des paramètres
#         self.declare_parameter('image_topic', '/camera/image_raw/compressed')
#         self.declare_parameter('camera_info_topic', '/camera/camera_info')
#         self.declare_parameter('marker_size', 0.068)
        
#         self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
#         self.info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
#         self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value

#         self.bridge = CvBridge()
#         self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

#         # Paramètres de détection OpenCV optimisés
#         self.parameters = cv2.aruco.DetectorParameters_create()
#         self.parameters.adaptiveThreshWinSizeMin = 3
#         self.parameters.adaptiveThreshWinSizeMax = 23
#         self.parameters.adaptiveThreshWinSizeStep = 10
#         self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

#         # Variables pour la calibration (Matrice K et Distorsion D)
#         self.camera_matrix = None
#         self.dist_coeffs = None
        
#         # Variables de tracking de fréquence
#         self.last_callback_time = time.perf_counter()
        
#         # Écoute unique des infos de calibration de la caméra
#         self.info_sub = self.create_subscription(
#             CameraInfo,
#             self.info_topic,
#             self.camera_info_callback,
#             10
#         )

#         # Publishers
#         self.marker_pub = self.create_publisher(ArucoMarkers, '/aruco_markers', 10)
#         self.image_pub = self.create_publisher(Image, '~/debug_image', 10)

#         # Souscription au flux compressé
#         self.image_sub = self.create_subscription(
#             CompressedImage,
#             self.image_topic,
#             self.compressed_image_callback,
#             10
#         )
        
#         self.get_logger().info(f"⚡ Nœud ArUco Pose 3D connecté sur : {self.image_topic}")

#     def camera_info_callback(self, msg):
#         """ Récupère les données de calibration une seule fois """
#         self.camera_matrix = np.array(msg.k).reshape((3, 3))
#         self.dist_coeffs = np.array(msg.d)
#         self.get_logger().info("✅ Paramètres de calibration caméra reçus et chargés.")
#         self.destroy_subscription(self.info_sub)

#     def compressed_image_callback(self, msg):
#         # 1. Mesure de la fréquence d'entrée réelle du topic compressed
#         t_start = time.perf_counter()
#         dt_callback = t_start - self.last_callback_time
#         self.last_callback_time = t_start
#         input_hz = 1.0 / dt_callback if dt_callback > 0 else 0.0

#         if self.camera_matrix is None:
#             return

#         try:
#             # 2. Temps de décodage JPEG -> Matrice OpenCV
#             t0 = time.perf_counter()
#             frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#             gray = cv2.equalizeHist(gray)
#             t_decode = (time.perf_counter() - t0) * 1000  # en ms

#             # 3. Temps de détection des marqueurs (Le suspect principal)
#             t1 = time.perf_counter()
#             corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
#             t_detect = (time.perf_counter() - t1) * 1000  # en ms

#             t_pose = 0.0
#             t_pub = 0.0
#             detected_status = "❌ AUCUN TAG"

#             if ids is not None and len(ids) > 0:
#                 detected_status = f"✅ TAG ID {ids.flatten().tolist()}"
                
#                 # 4. Temps d'estimation de pose et calculs géométriques
#                 t2 = time.perf_counter()
#                 markers_msg = ArucoMarkers()
#                 markers_msg.header.stamp = msg.header.stamp
#                 markers_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else "camera_link_optical"

#                 rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
#                     corners, self.marker_size, self.camera_matrix, self.dist_coeffs
#                 )

#                 cv2.aruco.drawDetectedMarkers(frame, corners, ids)

#                 for i in range(len(ids)):
#                     marker_id = int(ids[i][0])
#                     markers_msg.marker_ids.append(marker_id)
#                     cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)

#                     pose = Pose()
#                     pose.position.x = float(tvecs[i][0][0])
#                     pose.position.y = float(tvecs[i][0][1])
#                     pose.position.z = float(tvecs[i][0][2])

#                     r_matrix, _ = cv2.Rodrigues(rvecs[i])
#                     t = np.trace(r_matrix)
#                     if t > 0:
#                         M = np.sqrt(t + 1.0) * 2
#                         pose.orientation.w = 0.25 * M
#                         pose.orientation.x = (r_matrix[2, 1] - r_matrix[1, 2]) / M
#                         pose.orientation.y = (r_matrix[0, 2] - r_matrix[2, 0]) / M
#                         pose.orientation.z = (r_matrix[1, 0] - r_matrix[0, 1]) / M
#                     else:
#                         if (r_matrix[0, 0] > r_matrix[1, 1]) and (r_matrix[0, 0] > r_matrix[2, 2]):
#                             M = np.sqrt(1.0 + r_matrix[0, 0] - r_matrix[1, 1] - r_matrix[2, 2]) * 2
#                             pose.orientation.w = (r_matrix[2, 1] - r_matrix[1, 2]) / M
#                             pose.orientation.x = 0.25 * M
#                             pose.orientation.y = (r_matrix[0, 1] + r_matrix[1, 0]) / M
#                             pose.orientation.z = (r_matrix[0, 2] + r_matrix[2, 0]) / M
#                         elif r_matrix[1, 1] > r_matrix[2, 2]:
#                             M = np.sqrt(1.0 + r_matrix[1, 1] - r_matrix[0, 0] - r_matrix[2, 2]) * 2
#                             pose.orientation.w = (r_matrix[0, 2] - r_matrix[2, 0]) / M
#                             pose.orientation.x = (r_matrix[0, 1] + r_matrix[1, 0]) / M
#                             pose.orientation.y = 0.25 * M
#                             pose.orientation.z = (r_matrix[1, 2] + r_matrix[2, 1]) / M
#                         else:
#                             M = np.sqrt(1.0 + r_matrix[2, 2] - r_matrix[0, 0] - r_matrix[1, 1]) * 2
#                             pose.orientation.w = (r_matrix[1, 0] - r_matrix[0, 1]) / M
#                             pose.orientation.x = (r_matrix[0, 2] + r_matrix[2, 0]) / M
#                             pose.orientation.y = (r_matrix[1, 2] + r_matrix[2, 1]) / M
#                             pose.orientation.z = 0.25 * M

#                     markers_msg.poses.append(pose)
#                 t_pose = (time.perf_counter() - t2) * 1000

#                 # 5. Temps d'encodage et de publication de l'image brute de debug
#                 t3 = time.perf_counter()
#                 self.marker_pub.publish(markers_msg)
#                 debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
#                 self.image_pub.publish(debug_msg)
#                 t_pub = (time.perf_counter() - t3) * 1000

#             # 6. Log global condensé pour analyse immédiate
#             total_loop = t_decode + t_detect + t_pose + t_pub
#             self.get_logger().info(
#                 f"Fréq Entrée: {input_hz:.1f}Hz | Décode: {t_decode:.1f}ms | Détect: {t_detect:.1f}ms | "
#                 f"Pose: {t_pose:.1f}ms | Pub: {t_pub:.1f}ms | Total: {total_loop:.1f}ms | {detected_status}"
#             )

#         except CvBridgeError as e:
#             self.get_logger().error(f"Erreur : {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = CompressedArucoNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage, CameraInfo
# from geometry_msgs.msg import Pose
# from ros2_aruco_interfaces.msg import ArucoMarkers
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np
# import time

# class CompressedArucoNode(Node):
#     def __init__(self):
#         super().__init__('compressed_aruco_node')
        
#         # Déclaration des paramètres
#         self.declare_parameter('image_topic', '/camera/image_raw/compressed')
#         self.declare_parameter('camera_info_topic', '/camera/camera_info')
#         self.declare_parameter('marker_size', 0.068)
#         self.declare_parameter('enable_debug_image', True) # Switch pour couper la vidéo si besoin

#         self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
#         self.info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
#         self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
#         self.enable_debug_image = self.get_parameter('enable_debug_image').get_parameter_value().bool_value

#         self.bridge = CvBridge()
#         self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

#         # Paramètres de détection OpenCV optimisés
#         self.parameters = cv2.aruco.DetectorParameters_create()
#         self.parameters.adaptiveThreshWinSizeMin = 3
#         self.parameters.adaptiveThreshWinSizeMax = 23
#         self.parameters.adaptiveThreshWinSizeStep = 10
#         self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

#         self.camera_matrix = None
#         self.dist_coeffs = None
#         self.last_callback_time = time.perf_counter()
        
#         # Écoute unique des infos de calibration
#         self.info_sub = self.create_subscription(
#             CameraInfo, self.info_topic, self.camera_info_callback, 10
#         )

#         # Publishers
#         self.marker_pub = self.create_publisher(ArucoMarkers, '/aruco_markers', 10)
        
#         # Correction : On publie la debug image au format COMPRESSED
#         if self.enable_debug_image:
#             self.image_pub = self.create_publisher(CompressedImage, '~/debug_image/compressed', 10)

#         # Souscription au flux compressé
#         self.image_sub = self.create_subscription(
#             CompressedImage, self.image_topic, self.compressed_image_callback, 10
#         )
        
#         self.get_logger().info(f"⚡ Nœud ArUco Pose 3D connecté sur : {self.image_topic}")

#     def camera_info_callback(self, msg):
#         self.camera_matrix = np.array(msg.k).reshape((3, 3))
#         self.dist_coeffs = np.array(msg.d)
#         self.get_logger().info("✅ Paramètres de calibration caméra reçus et chargés.")
#         self.destroy_subscription(self.info_sub)

#     def compressed_image_callback(self, msg):
#         t_start = time.perf_counter()
#         dt_callback = t_start - self.last_callback_time
#         self.last_callback_time = t_start
#         input_hz = 1.0 / dt_callback if dt_callback > 0 else 0.0

#         if self.camera_matrix is None:
#             return

#         try:
#             t0 = time.perf_counter()
#             frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#             gray = cv2.equalizeHist(gray)
#             t_decode = (time.perf_counter() - t0) * 1000

#             t1 = time.perf_counter()
#             corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
#             t_detect = (time.perf_counter() - t1) * 1000

#             t_pose = 0.0
#             t_pub = 0.0
#             detected_status = "❌ AUCUN TAG"

#             if ids is not None and len(ids) > 0:
#                 detected_status = f"✅ TAG ID {ids.flatten().tolist()}"
                
#                 t2 = time.perf_counter()
#                 markers_msg = ArucoMarkers()
#                 markers_msg.header.stamp = msg.header.stamp
#                 markers_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else "camera_link_optical"

#                 rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
#                     corners, self.marker_size, self.camera_matrix, self.dist_coeffs
#                 )

#                 if self.enable_debug_image:
#                     cv2.aruco.drawDetectedMarkers(frame, corners, ids)

#                 for i in range(len(ids)):
#                     marker_id = int(ids[i][0])
#                     markers_msg.marker_ids.append(marker_id)
                    
#                     if self.enable_debug_image:
#                         cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)

#                     pose = Pose()
#                     pose.position.x = float(tvecs[i][0][0])
#                     pose.position.y = float(tvecs[i][0][1])
#                     pose.position.z = float(tvecs[i][0][2])

#                     r_matrix, _ = cv2.Rodrigues(rvecs[i])
#                     t = np.trace(r_matrix)
#                     if t > 0:
#                         M = np.sqrt(t + 1.0) * 2
#                         pose.orientation.w = 0.25 * M
#                         pose.orientation.x = (r_matrix[2, 1] - r_matrix[1, 2]) / M
#                         pose.orientation.y = (r_matrix[0, 2] - r_matrix[2, 0]) / M
#                         pose.orientation.z = (r_matrix[1, 0] - r_matrix[0, 1]) / M
#                     else:
#                         if (r_matrix[0, 0] > r_matrix[1, 1]) and (r_matrix[0, 0] > r_matrix[2, 2]):
#                             M = np.sqrt(1.0 + r_matrix[0, 0] - r_matrix[1, 1] - r_matrix[2, 2]) * 2
#                             pose.orientation.w = (r_matrix[2, 1] - r_matrix[1, 2]) / M
#                             pose.orientation.x = 0.25 * M
#                             pose.orientation.y = (r_matrix[0, 1] + r_matrix[1, 0]) / M
#                             pose.orientation.z = (r_matrix[0, 2] + r_matrix[2, 0]) / M
#                         elif r_matrix[1, 1] > r_matrix[2, 2]:
#                             M = np.sqrt(1.0 + r_matrix[1, 1] - r_matrix[0, 0] - r_matrix[2, 2]) * 2
#                             pose.orientation.w = (r_matrix[0, 2] - r_matrix[2, 0]) / M
#                             pose.orientation.x = (r_matrix[0, 1] + r_matrix[1, 0]) / M
#                             pose.orientation.y = 0.25 * M
#                             pose.orientation.z = (r_matrix[1, 2] + r_matrix[2, 1]) / M
#                         else:
#                             M = np.sqrt(1.0 + r_matrix[2, 2] - r_matrix[0, 0] - r_matrix[1, 1]) * 2
#                             pose.orientation.w = (r_matrix[1, 0] - r_matrix[0, 1]) / M
#                             pose.orientation.x = (r_matrix[0, 2] + r_matrix[2, 0]) / M
#                             pose.orientation.y = (r_matrix[1, 2] + r_matrix[2, 1]) / M
#                             pose.orientation.z = 0.25 * M

#                     markers_msg.poses.append(pose)
#                 t_pose = (time.perf_counter() - t2) * 1000

#                 t3 = time.perf_counter()
#                 # Publication immédiate des coordonnées numériques
#                 self.marker_pub.publish(markers_msg)
                
#                 # Publication de l'image de débug compressée en JPEG (Ultra-rapide)
#                 if self.enable_debug_image:
#                     _, jpeg_data = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
#                     debug_msg = CompressedImage()
#                     debug_msg.header.stamp = msg.header.stamp
#                     debug_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else "camera_link_optical"
#                     debug_msg.format = "jpeg"
#                     debug_msg.data = jpeg_data.tobytes()
#                     self.image_pub.publish(debug_msg)
#                 t_pub = (time.perf_counter() - t3) * 1000

#             total_loop = t_decode + t_detect + t_pose + t_pub
#             self.get_logger().info(
#                 f"Fréq Entrée: {input_hz:.1f}Hz | Décode: {t_decode:.1f}ms | Détect: {t_detect:.1f}ms | "
#                 f"Pose: {t_pose:.1f}ms | Pub: {t_pub:.1f}ms | Total: {total_loop:.1f}ms | {detected_status}"
#             )

#         except CvBridgeError as e:
#             self.get_logger().error(f"Erreur : {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = CompressedArucoNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage, Image
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np

# class CompressedArucoNode(Node):
#     def __init__(self):
#         super().__init__('compressed_aruco_node')
        
#         # Déclaration et récupération des paramètres ROS 2
#         self.declare_parameter('image_topic', '/camera/image_raw/compressed')
#         self.declare_parameter('marker_size', 0.068)
        
#         self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
#         self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value

#         self.bridge = CvBridge()
        
#         # Initialisation directe et unique du dictionnaire ORIGINAL
#         self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

#         # Configuration des paramètres du détecteur OpenCV
#         self.parameters = cv2.aruco.DetectorParameters_create()
#         self.parameters.adaptiveThreshWinSizeMin = 3
#         self.parameters.adaptiveThreshWinSizeMax = 23
#         self.parameters.adaptiveThreshWinSizeStep = 10
#         self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

#         # Publisher pour visualiser le flux de debug dans RViz
#         self.image_pub = self.create_publisher(Image, '~/debug_image', 10)

#         # Souscription unique et stricte au format compressé
#         self.image_sub = self.create_subscription(
#             CompressedImage,
#             self.image_topic,
#             self.compressed_image_callback,
#             10
#         )
        
#         self.get_logger().info(f"⚡ Nœud ArUco (DICT_ORIGINAL) connecté sur : {self.image_topic}")

#     def compressed_image_callback(self, msg):
#         try:
#             #     # 1. Loggez le format exact écrit dans le message ROS 2
#             # self.get_logger().info(f"Format reçu du robot : '{msg.format}'", once=True)

#             # # Décodage
#             # frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
#             # # 2. Inspectez la structure réelle de la matrice OpenCV générée
#             # self.get_logger().info(f"Shape OpenCV : {frame.shape} | Type : {frame.dtype}", once=True)

#             # Décodage direct du buffer binaire JPEG/PNG, indépendant de CvBridge
#             np_arr = np.frombuffer(msg.data, np.uint8)
#             frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # Force le format BGR 3 canaux
            
#             if frame is None:
#                 self.get_logger().error("Échec critique du décodage de l'image compressée")
#                 return
            
#             # Prétraitement pour améliorer le contraste du tag
#             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#             gray = cv2.equalizeHist(gray)

#             # Détection ArUco
#             corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
            
#             if ids is not None and len(ids) > 0:
#                 cv2.aruco.drawDetectedMarkers(frame, corners, ids)
#                 for i in range(len(ids)):
#                     marker_id = int(ids[i][0])
#                     self.get_logger().info(f"🎯 Tag détecté ! ID: {marker_id}")
#                     cv2.putText(frame, f"ID: {marker_id}", (10, 30),
#                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
#                 # Envoi de l'image modifiée sur le topic de debug
#                 debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
#                 self.image_pub.publish(debug_msg)

#         except CvBridgeError as e:
#             self.get_logger().error(f"Erreur de décodage/conversion : {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = CompressedArucoNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class CompressedArucoNode(Node):
    def __init__(self):
        super().__init__('compressed_aruco_node')
        
        # 1. Paramètres ROS 2
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('marker_size', 0.068)
        self.declare_parameter('parent_frame', 'camera_link_optical')
        self.declare_parameter('enable_debug', False) # Désactivé par défaut pour les performances

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.enable_debug = self.get_parameter('enable_debug').get_parameter_value().bool_value

        # 2. Outils OpenCV et TF
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

        # Configuration du détecteur
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.camera_matrix = None
        self.dist_coeffs = None
        
        # 3. Souscriptions et Publishers
        self.info_sub = self.create_subscription(
            CameraInfo, self.info_topic, self.camera_info_callback, 10
        )
        
        self.pose_array_pub = self.create_publisher(PoseArray, '/aruco_marker_poses', 10)
        
        # Instanciation conditionnelle du publisher d'image
        if self.enable_debug:
            self.image_pub = self.create_publisher(Image, '~/debug_image', 10)
            self.get_logger().info("⚠️ Mode DEBUG actif (consommation CPU supérieure).")

        self.image_sub = self.create_subscription(
            CompressedImage, self.image_topic, self.compressed_image_callback, 10
        )
        
        self.get_logger().info(f"⚡ Nœud ArUco actif. Mode pure performance. Parent : {self.parent_frame}")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("✅ Calibration chargée. Fermeture du souscripteur info.")
        self.destroy_subscription(self.info_sub)

    def compressed_image_callback(self, msg):
        if self.camera_matrix is None:
            return

        try:
            # Décodage de l'image (requis pour la détection dans tous les cas)
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # Force le format BGR 3 canaux
            
            if frame is None:
                self.get_logger().error("Échec critique du décodage de l'image compressée")
                return
            
            #frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Égalisation d'histogramme
            gray = cv2.equalizeHist(gray)

            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
            
            if ids is not None and len(ids) > 0:
                pose_array_msg = PoseArray()
                pose_array_msg.header.stamp = msg.header.stamp
                pose_array_msg.header.frame_id = self.parent_frame

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                )

                # Le dessin sur l'image n'est exécuté que si le débug est actif
                if self.enable_debug:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    
                    if self.enable_debug:
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)

                    # Extraction de la Pose
                    pose = Pose()
                    pose.position.x = float(tvecs[i][0][0])
                    pose.position.y = float(tvecs[i][0][1])
                    pose.position.z = float(tvecs[i][0][2])

                    # Conversion Rodrigues -> Quaternion
                    r_matrix, _ = cv2.Rodrigues(rvecs[i])
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

                    pose_array_msg.poses.append(pose)

                    # Diffusion de la TF
                    tf_msg = TransformStamped()
                    tf_msg.header.stamp = msg.header.stamp
                    tf_msg.header.frame_id = self.parent_frame
                    tf_msg.child_frame_id = f'aruco_marker_{marker_id}'
                    tf_msg.transform.translation.x = pose.position.x
                    tf_msg.transform.translation.y = pose.position.y
                    tf_msg.transform.translation.z = pose.position.z
                    tf_msg.transform.rotation = pose.orientation
                    self.tf_broadcaster.sendTransform(tf_msg)

                # Publication des Poses (Léger)
                self.pose_array_pub.publish(pose_array_msg)
                
                # Publication de la vidéo uniquement si demandée
                if self.enable_debug:
                    debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    self.image_pub.publish(debug_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"Erreur callback : {e}")

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