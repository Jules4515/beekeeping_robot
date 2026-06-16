#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ArucoTFBroadcaster(Node):
    def __init__(self):
        super().__init__('aruco_tf_broadcaster')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Nom du repère parent cible (à aligner avec ton URDF)
        self.declare_parameter('parent_frame', 'camera_link_optical')
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.aruco_callback,
            10
        )
        self.get_logger().info(f"Diffuseur TF ArUco actif. Parent imposé : {self.parent_frame}")

    def aruco_callback(self, msg):
        for i, marker_id in enumerate(msg.marker_ids):
            t = TransformStamped()
            
            # Synchronisation temporelle avec la capture de l'image
            t.header.stamp = msg.header.stamp
            
            # FORCE le repère parent pour éviter les dérives
            t.header.frame_id = self.parent_frame
            
            # Identifiant unique de la TF du tag
            t.child_frame_id = f'aruco_marker_{marker_id}'
            
            # Coordonnées spatiales lues par le nœud ArUco
            t.transform.translation.x = msg.poses[i].position.x
            t.transform.translation.y = msg.poses[i].position.y
            t.transform.translation.z = msg.poses[i].position.z
            
            # Orientation (quaternion)
            t.transform.rotation = msg.poses[i].orientation
            
            # Envoi dans l'arbre global de ROS 2
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()