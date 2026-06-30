# #!/usr/bin/env python3
# """
# GPS Monitor - Affiche la qualité du fix GPS
# Lit /gps/status (String) et /heading_imu (Imu)
# """

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from std_msgs.msg import String
# import math
# import numpy as np

# class GPSMonitor(Node):
#     def __init__(self):
#         super().__init__('gps_monitor')
        
#         self.gps_status = 'ATTENTE...'
#         self.gps_emoji = '🔴'
#         self.gps_received = False
#         self.heading_received = False
#         self.current_yaw = 0.0
#         self.last_headings = []
#         self.heading_stable = False
        
#         # Subscribers
#         self.status_sub = self.create_subscription(String, '/gps/status', self.status_callback, 10)
#         self.heading_sub = self.create_subscription(Imu, '/heading_imu', self.heading_callback, 10)
        
#         # Timer 1 Hz
#         self.timer = self.create_timer(1.0, self.publish_status)
        
#         self.get_logger().info('📡 GPS Monitor démarré (écoute /gps/status)')
        
#     def status_callback(self, msg):
#         if not self.gps_received:
#             self.get_logger().info('📡 GPS connecté !')
#             self.gps_received = True
        
#         data = msg.data
        
#         if data == 'RTK_FIXED':
#             self.gps_status = 'RTK FIXED'
#             self.gps_emoji = '🟢'
#         elif data == 'DGPS':
#             self.gps_status = 'DGPS'
#             self.gps_emoji = '🟡'
#         elif data == 'GPS':
#             self.gps_status = 'GPS'
#             self.gps_emoji = '🟠'
#         else:
#             self.gps_status = 'PAS DE FIX'
#             self.gps_emoji = '🔴'
        
#         self.get_logger().info(
#             f'DEBUG GPS - {self.gps_emoji} {self.gps_status}',
#             throttle_duration_sec=5.0
#         )
        
#     def heading_callback(self, msg: Imu):
#         if not self.heading_received:
#             self.get_logger().info('🧭 Heading GPS connecté !')
#             self.heading_received = True
        
#         q = msg.orientation
#         yaw = math.degrees(2.0 * math.atan2(q.z, q.w))
#         if yaw < 0:
#             yaw += 360.0
#         self.current_yaw = yaw
        
#         self.last_headings.append(yaw)
#         if len(self.last_headings) > 20:
#             self.last_headings.pop(0)
#         if len(self.last_headings) >= 5:
#             self.heading_stable = np.var(self.last_headings) < 10.0
    
#     def publish_status(self):
#         heading = f"{self.current_yaw:.0f}°" if self.heading_received else "?"
#         stable = "✓" if self.heading_stable else "~"
        
#         self.get_logger().info(
#             f'{self.gps_emoji} GPS: {self.gps_status} {stable}',
#             throttle_duration_sec=1.0
#         )

# def main(args=None):
#     rclpy.init(args=args)
#     node = GPSMonitor()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
GPS Monitor - Affiche la qualité du fix GPS
Lit /gps/status (String) et /heading_imu (Imu)
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import math
import numpy as np

class GPSMonitor(Node):
    def __init__(self):
        super().__init__('gps_monitor')
        
        self.gps_status = 'ATTENTE...'
        self.gps_value = -1
        self.gps_precision = 'ATTENTE...'
        self.gps_received = False
        self.heading_received = False
        self.current_yaw = 0.0
        self.last_headings = []
        self.heading_stable = False
        
        # Subscribers
        self.status_sub = self.create_subscription(String, '/gps/status', self.status_callback, 10)
        self.heading_sub = self.create_subscription(Imu, '/heading_imu', self.heading_callback, 10)
        
        # Timer 1 Hz
        self.timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('📡 GPS Monitor démarré (écoute /gps/status)')
        
    def status_callback(self, msg):
        if not self.gps_received:
            self.get_logger().info('📡 GPS connecté !')
            self.gps_received = True
        
        data = msg.data
        
        if data == 'RTK_FIXED':
            self.gps_status = 'RTK FIXED'
            self.gps_value = 4
            self.gps_precision = '1-3 cm'
        elif data == 'RTK_FLOAT':
            self.gps_status = 'RTK FLOAT'
            self.gps_value = 5
            self.gps_precision = '20 cm - 1 m'
        elif data == 'DGPS':
            self.gps_status = 'DGPS'
            self.gps_value = 2
            self.gps_precision = '0.5-2 m'
        elif data == 'GPS':
            self.gps_status = 'GPS'
            self.gps_value = 1
            self.gps_precision = '2-5 m'
        else:
            self.gps_status = 'PAS DE FIX'
            self.gps_value = 0
            self.gps_precision = 'AUCUNE'
        
        self.get_logger().info(
            f'DEBUG GPS - {self.gps_value} ({self.gps_status}) → {self.gps_precision}',
            throttle_duration_sec=5.0
        )
        
    def heading_callback(self, msg: Imu):
        if not self.heading_received:
            self.get_logger().info('🧭 Heading GPS connecté !')
            self.heading_received = True
        
        q = msg.orientation
        yaw = math.degrees(2.0 * math.atan2(q.z, q.w))
        if yaw < 0:
            yaw += 360.0
        self.current_yaw = yaw
        
        self.last_headings.append(yaw)
        if len(self.last_headings) > 20:
            self.last_headings.pop(0)
        if len(self.last_headings) >= 5:
            self.heading_stable = np.var(self.last_headings) < 10.0
    
    def publish_status(self):
        heading = f"{self.current_yaw:.0f}°" if self.heading_received else "?"
        stable = "✓" if self.heading_stable else "~"
        
        self.get_logger().info(
            f'GPS: {self.gps_value} | Précision: {self.gps_precision}',
            throttle_duration_sec=1.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = GPSMonitor()
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