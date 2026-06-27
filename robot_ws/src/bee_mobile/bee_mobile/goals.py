#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose2D
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float64MultiArray

class Nav2GoalManager(Node):
    def __init__(self):
        super().__init__('nav2_goal_manager')
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # --- AJOUT : Souscription au topic de commande ---
        self.goal_sub = self.create_subscription(
            Pose2D,
            '/custom_goal',
            self.goal_topic_callback,
            10
        )
        
        self.wheel_pubs = {}
        wheel_names = ['front_left', 'front_right', 'rear_left', 'rear_right']
        for name in wheel_names:
            self.wheel_pubs[name] = self.create_publisher(
                Float64MultiArray, 
                f'mobile/wheel_{name}/motor_speed', 
                10
            )
            
        self.get_logger().info('Prêt. En attente de coordonnées sur le topic /custom_docking_goal ...')

    def goal_topic_callback(self, msg):
        """Reçoit le Pose2D (x, y, theta) et déclenche l'action."""
        self.get_logger().info('Nouveau goal reçu depuis le terminal.')
        
        # Extraction directe, aucune conversion trigonométrique requise ici
        x = msg.x
        y = msg.y
        yaw = msg.theta
        
        self.send_goal(x, y, yaw)

    def send_goal(self, x, y, yaw):
        self.nav_to_pose_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        half_yaw = yaw * 0.5
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(half_yaw)
        goal_msg.pose.pose.orientation.w = math.cos(half_yaw)

        self.get_logger().info(f'Envoi du goal à Nav2: X={x:.2f}, Y={y:.2f}, Yaw={yaw:.2f} rad')
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejeté par Nav2.')
            return

        self.get_logger().info('Goal accepté. Navigation en cours...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        # Status 4 = SUCCEEDED
        if status == 4:
            self.get_logger().info('Goal atteint ! Activation du freinage.')
            self.force_stop_motors()
        else:
            self.get_logger().warn(f'Goal annulé ou échoué. Code de statut : {status}')
            
        # Attention : Retrait de rclpy.shutdown() ici pour que le nœud reste en écoute

    def force_stop_motors(self):
        for i in range(3):
            stop_msg = Float64MultiArray(data=[0.0, 0.0])
            for name, pub in self.wheel_pubs.items():
                pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.force_stop_motors()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()