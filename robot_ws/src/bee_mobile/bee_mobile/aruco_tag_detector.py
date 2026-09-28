#!/usr/bin/env python3

#
# Aruco Tag Detector node for ROS2
#
# This node detects a specific ArUco marker from the original ArUco dictionary (DICT_ARUCO_ORIGINAL) in a camera
# feed. It publishes the marker's pose as a TransformStamped message (TF) and optionally as a PoseArray (on a 
# specific topic). It can also publish a debug image with the detected marker and its axes drawn on it.
# The node uses OpenCV's ArUco module for detection and pose estimation. 
# It requires camera calibration parameters to be provided via a CameraInfo topic.
#

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ArucoTagDetector(Node):
    def __init__(self):
        super().__init__('aruco_tag_detector')

        # --- ROS 2 parameters ---

        # Image and camera info topics
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')

        # ArUco marker side length in meters (on the physical marker)
        self.declare_parameter('marker_size', 0.068)

        # Frame in which the marker pose is published
        self.declare_parameter('parent_frame', 'camera_link')

        # Debug image mode, disabled by default to reduce CPU usage
        self.declare_parameter('enable_debug_image', False)

        # ArUco ID to detect
        self.declare_parameter('target_id', 91)

        # Whether to publish a PoseArray of detected markers
        self.declare_parameter('publish_pose_array', False)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.enable_debug_image = self.get_parameter('enable_debug_image').get_parameter_value().bool_value
        self.target_id = self.get_parameter('target_id').get_parameter_value().integer_value
        self.publish_pose_array = self.get_parameter('publish_pose_array').get_parameter_value().bool_value

        # --- OpenCV and ROS 2 initialization ---
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

        # --- ArUco detection parameters ---
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.camera_matrix = None
        self.dist_coeffs = None
        
        # --- Subscriptions and publishers ---
        self.info_sub = self.create_subscription(
            CameraInfo, self.info_topic, self.camera_info_callback, 10
        )
        
        self.pose_array_pub = self.create_publisher(PoseArray, '/aruco_marker_poses', 10)
        
        if self.enable_debug_image:
            self.image_pub = self.create_publisher(Image, '~/debug_image', 10)
            self.get_logger().info("⚠️ DEBUG mode enabled (higher CPU usage).")

        self.image_sub = self.create_subscription(
            CompressedImage, self.image_topic, self.compressed_image_callback, 10
        )
        
        self.get_logger().info(f"ArUco detector active. Parent: {self.parent_frame}")

    # Load camera calibration parameters
    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("✅ Camera calibration loaded. Shutting down the camera info subscription.")
        self.destroy_subscription(self.info_sub)

    # Process compressed image messages
    def compressed_image_callback(self, msg):
        if self.camera_matrix is None:
            return

        try:
            # Decode the compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Force 3-channel BGR format
            
            if frame is None:
                self.get_logger().error("Failed to decode the compressed image.")
                return
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Equalize the grayscale histogram
            gray = cv2.equalizeHist(gray)

            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
            
            if ids is not None and len(ids) > 0:
                # --- Filter by target ID ---
                target_idx = None
                for i in range(len(ids)):
                    if int(ids[i][0]) == self.target_id:
                        target_idx = i
                        break
                
                # Stop processing if the target marker is not in the frame
                if target_idx is None:
                    # Publish the debug image anyway when debug mode is enabled
                    if self.enable_debug_image:
                        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                        self.image_pub.publish(debug_msg)
                    return
                
                # Keep only the target marker
                corners = (corners[target_idx],)
                ids = np.array([ids[target_idx]], dtype=np.int32)

                if self.publish_pose_array:
                    pose_array_msg = PoseArray()
                    pose_array_msg.header.stamp = msg.header.stamp
                    pose_array_msg.header.frame_id = self.parent_frame

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                )

                # Draw annotations only in debug mode
                if self.enable_debug_image:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    
                    if self.enable_debug_image:
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)

                    # Extract the marker pose
                    pose = Pose()
                    pose.position.x = float(tvecs[i][0][0])
                    pose.position.y = float(tvecs[i][0][1])
                    pose.position.z = float(tvecs[i][0][2])

                    # Convert the Rodrigues rotation vector to a quaternion
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

                    if self.publish_pose_array:
                        pose_array_msg.poses.append(pose)

                    # Broadcast the transform
                    tf_msg = TransformStamped()
                    tf_msg.header.stamp = msg.header.stamp
                    tf_msg.header.frame_id = self.parent_frame
                    tf_msg.child_frame_id = f'aruco_marker_{marker_id}'
                    tf_msg.transform.translation.x = pose.position.x
                    tf_msg.transform.translation.y = pose.position.y
                    tf_msg.transform.translation.z = pose.position.z
                    tf_msg.transform.rotation = pose.orientation
                    self.tf_broadcaster.sendTransform(tf_msg)

                if self.publish_pose_array:
                    self.pose_array_pub.publish(pose_array_msg)
                
                if self.enable_debug_image:
                    debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    self.image_pub.publish(debug_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"Callback error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTagDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print(f"[INFO] [{node.get_name()}]: Shutdown requested by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()