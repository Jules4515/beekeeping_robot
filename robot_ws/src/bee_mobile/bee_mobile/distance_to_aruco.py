#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros

class MarkerXNode(Node):
    def __init__(self):
        super().__init__('marker_x_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            t = self.tf_buffer.lookup_transform('base_link', 'aruco_marker_91', now)
            dx = t.transform.translation.x
            self.get_logger().info(f"dx (signed) = {dx:.3f} m, |dx| = {abs(dx):.3f} m")
        except Exception as e:
            self.get_logger().info(f"Transform unavailable: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerXNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()