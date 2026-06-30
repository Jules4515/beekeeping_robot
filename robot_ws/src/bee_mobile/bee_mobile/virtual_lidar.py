# #!/usr/bin/env python3
# """Lidar virtuel qui publie un scan vide pour le collision monitor"""
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# import math

# class VirtualLidar(Node):
#     def __init__(self):
#         super().__init__('virtual_lidar')
#         # Publie sur un topic DÉDIÉ, sans toucher à /scan
#         self.publisher = self.create_publisher(LaserScan, '/virtual_scan', 10)
#         self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz
#         self.get_logger().info('Virtual Lidar started on /virtual_scan')
        
#     def publish_scan(self):
#         scan = LaserScan()
#         scan.header.stamp = self.get_clock().now().to_msg()
#         scan.header.frame_id = 'base_link'
#         scan.angle_min = -math.pi
#         scan.angle_max = math.pi
#         scan.angle_increment = math.pi / 180  # 1 degré = 360 points
#         scan.time_increment = 0.0
#         scan.scan_time = 0.1
#         scan.range_min = 0.05
#         scan.range_max = 30.0
#         # scan complètement vide (désert)
#         scan.ranges = [float('inf')] * 360
#         scan.intensities = [0.0] * 360
#         self.publisher.publish(scan)

# def main():
#     rclpy.init()
#     rclpy.spin(VirtualLidar())
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
Virtual LiDAR Node
Publie un LaserScan vide (pas d'obstacles) pour satisfaire le collision monitor
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan
import math

class VirtualLidar(Node):
    def __init__(self):
        super().__init__('virtual_lidar')
        
        # Paramètre : fréquence de publication
        self.declare_parameter('publish_rate', 10.0)
        publish_rate = self.get_parameter('publish_rate').value
        
        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Timer
        self.timer = self.create_timer(1.0/publish_rate, self.publish_scan)
        
        self.get_logger().info('Virtual LiDAR démarré - publie un scan vide')
    
    def publish_scan(self):
        """Publie un LaserScan sans obstacles"""
        msg = LaserScan()
        
        # Timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # Ou 'laser' si vous avez une TF
        
        # Configuration du scan (360°)
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.pi / 180  # 1° de résolution
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 30.0  # Portée max fictive
        
        # Toutes les mesures à la distance max = pas d'obstacle
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = [float('inf')] * num_readings  # inf = pas d'obstacle
        msg.intensities = [0.0] * num_readings
        
        self.scan_pub.publish(msg)

def main():
    rclpy.init()
    node = VirtualLidar()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print(f"\n[INFO] [{node.get_name()}]: Shutdown requested by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()