# #!/usr/bin/env python3
# """
# GPS Health Monitor pour ROS 2
# Surveille la qualité du signal GPS et active/désactive la correction GPS
# pour éviter les conflits avec AMCL.

# Politique :
# - Si fix GPS de qualité RTK (status >= 2) : active la correction GPS
# - Si perte de GPS ou qualité dégradée : désactive la correction GPS
# """

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from geometry_msgs.msg import TwistWithCovarianceStamped
# from std_srvs.srv import SetBool
# from std_msgs.msg import Bool

# class GPSHealthMonitor(Node):
#     def __init__(self):
#         super().__init__('gps_health_monitor')
        
#         # Paramètres
#         self.declare_parameter('min_gps_status', 2)  # 0=no fix, 1=GPS, 2=DGPS, 4=RTK fixed
#         self.declare_parameter('gps_timeout', 2.0)    # secondes avant de considérer le GPS perdu
#         self.declare_parameter('publish_health', True)
        
#         self.min_status = self.get_parameter('min_gps_status').value
#         self.timeout = self.get_parameter('gps_timeout').value
        
#         # État du GPS
#         self.last_gps_time = self.get_clock().now()
#         self.gps_healthy = False
#         self.current_status = -1
        
#         # Subscribers
#         self.fix_sub = self.create_subscription(
#             NavSatFix,
#             '/fix',
#             self.fix_callback,
#             10
#         )
        
#         # Publisher pour l'état de santé du GPS
#         self.health_pub = self.create_publisher(Bool, '/gps/healthy', 10)
        
#         # Timer pour vérifier le timeout GPS
#         self.timer = self.create_timer(0.5, self.check_gps_timeout)
        
#         # Service pour activer/désactiver manuellement
#         self.srv = self.create_service(SetBool, '~/enable', self.enable_callback)
        
#         # Client pour activer/désactiver navsat_transform_node
#         # (Alternative : on publie juste l'état et un autre nœud gère)
        
#         self.get_logger().info('GPS Health Monitor démarré')
        
#     def fix_callback(self, msg: NavSatFix):
#         """Callback pour les messages GPS Fix"""
#         self.last_gps_time = self.get_clock().now()
#         old_healthy = self.gps_healthy
#         old_status = self.current_status
        
#         self.current_status = msg.status.status
        
#         # Vérifier si le statut GPS est suffisant
#         # status >= 0 : fix valide (0 = no fix, 1 = GPS, 2 = DGPS, 4 = RTK)
#         # status = -1 : pas de fix
#         if msg.status.status >= self.min_status and msg.status.status > 0:
#             self.gps_healthy = True
#         else:
#             self.gps_healthy = False
            
#         # Log si changement d'état
#         if old_healthy != self.gps_healthy or old_status != self.current_status:
#             status_text = self.get_status_text(self.current_status)
#             if self.gps_healthy:
#                 self.get_logger().info(f'GPS HEALTHY - Status: {status_text}')
#             else:
#                 self.get_logger().warn(f'GPS UNHEALTHY - Status: {status_text}')
                
#         # Publier l'état
#         if self.get_parameter('publish_health').value:
#             msg = Bool()
#             msg.data = self.gps_healthy
#             self.health_pub.publish(msg)
            
#     def check_gps_timeout(self):
#         """Vérifie si le GPS n'a pas envoyé de données depuis trop longtemps"""
#         now = self.get_clock().now()
#         elapsed = (now - self.last_gps_time).nanoseconds / 1e9
        
#         if elapsed > self.timeout and self.gps_healthy:
#             self.get_logger().warn(f'GPS TIMEOUT après {elapsed:.1f}s')
#             self.gps_healthy = False
#             msg = Bool()
#             msg.data = False
#             self.health_pub.publish(msg)
            
#     def enable_callback(self, request, response):
#         """Service pour activer/désactiver manuellement"""
#         self.gps_healthy = request.data
#         response.success = True
#         response.message = f'GPS health set to {self.gps_healthy}'
#         return response
        
#     def get_status_text(self, status):
#         """Convertit le code status GPS en texte lisible"""
#         status_map = {
#             -1: 'NO FIX',
#             0: 'NO FIX',
#             1: 'GPS FIX',
#             2: 'DGPS FIX',
#             4: 'RTK FIXED',
#             5: 'RTK FLOAT'
#         }
#         return status_map.get(status, f'UNKNOWN ({status})')

# def main(args=None):
#     rclpy.init(args=args)
#     node = GPSHealthMonitor()
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
"""
GPS Health Monitor pour ROS 2
Surveille la qualité du signal GPS et active/désactive la correction GPS
pour éviter les conflits avec AMCL.

Politique :
- Si fix GPS de qualité RTK/DGPS : active la correction GPS
- Si perte de GPS ou qualité dégradée : désactive la correction GPS

Sources surveillées :
- /gps/status (String) : source prioritaire
- /fix (NavSatFix) : fallback si /gps/status est silencieux
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import SetBool
from std_msgs.msg import Bool, String

class GPSHealthMonitor(Node):
    def __init__(self):
        super().__init__('gps_health_monitor')
        
        # Paramètres
        self.declare_parameter('min_gps_status', 2)  # 0=no fix, 1=GPS, 2=DGPS, 4=RTK fixed
        self.declare_parameter('gps_timeout', 2.0)    # secondes avant de considérer le GPS perdu
        self.declare_parameter('publish_health', True)
        self.declare_parameter('use_gps_status_topic', True)  # Priorité à /gps/status
        
        self.min_status = self.get_parameter('min_gps_status').value
        self.timeout = self.get_parameter('gps_timeout').value
        self.use_gps_status = self.get_parameter('use_gps_status_topic').value
        
        # État du GPS
        self.last_gps_status_time = self.get_clock().now()
        self.last_fix_time = self.get_clock().now()
        self.gps_healthy = False
        self.current_status = -1
        self.current_status_text = 'ATTENTE...'
        self.first_message = True  # Pour le message initial
        self.initialized = False
        
        # Subscribers - les deux sources
        self.status_sub = self.create_subscription(
            String,
            '/gps/status',
            self.gps_status_callback,
            10
        )
        
        self.fix_sub = self.create_subscription(
            NavSatFix,
            '/fix',
            self.fix_callback,
            10
        )
        
        # Publisher pour l'état de santé du GPS
        self.health_pub = self.create_publisher(Bool, '/gps/healthy', 10)
        
        # Timer pour vérifier le timeout GPS
        self.timer = self.create_timer(0.5, self.check_gps_timeout)
        
        # Service pour activer/désactiver manuellement
        self.srv = self.create_service(SetBool, '~/enable', self.enable_callback)
        
        self.get_logger().info('GPS Health Monitor démarré - en attente des données GPS...')
        
    def gps_status_callback(self, msg: String):
        """Callback prioritaire pour /gps/status (String)"""
        self.last_gps_status_time = self.get_clock().now()
        data = msg.data
        
        # Mapping String → code numérique (pour compatibilité)
        status_map = {
            'RTK_FIXED': 4,
            'RTK_FLOAT': 5,
            'DGPS': 2,
            'GPS': 1,
            'NO_FIX': 0,
            'PAS DE FIX': 0,
            'ATTENTE...': -1
        }
        
        new_status = status_map.get(data, -1)
        self.update_gps_state(new_status, data)
        
    def fix_callback(self, msg: NavSatFix):
        """Callback secondaire pour /fix (NavSatFix)"""
        self.last_fix_time = self.get_clock().now()
        
        # Ne traiter /fix que si /gps/status n'est pas utilisé ou est silencieux
        if not self.use_gps_status:
            new_status = msg.status.status
            self.update_gps_state(new_status, self.get_status_text(new_status))
    
    def update_gps_state(self, status_code, status_text):
        """Met à jour l'état GPS avec le code et le texte"""
        old_healthy = self.gps_healthy
        old_status = self.current_status
        
        self.current_status = status_code
        self.current_status_text = status_text
        
        # Vérifier si le statut GPS est suffisant
        if status_code >= self.min_status and status_code > 0:
            self.gps_healthy = True
        else:
            self.gps_healthy = False
        
        # Premier message d'initialisation
        if self.first_message and status_code != -1:
            self.first_message = False
            self.initialized = True
            if self.gps_healthy:
                self.get_logger().info(
                    f'✅ GPS initialisé - Status: {status_text} (code: {status_code}) - Précision: {self.get_precision(status_code)}'
                )
            else:
                self.get_logger().warn(
                    f'⚠️  GPS initialisé - Status: {status_text} (code: {status_code}) - Qualité insuffisante'
                )
        # Messages uniquement lors des changements d'état
        elif self.initialized and (old_healthy != self.gps_healthy or old_status != self.current_status):
            if self.gps_healthy and not old_healthy:
                self.get_logger().info(
                    f'✅ GPS OK - Status: {status_text} (code: {status_code}) - Précision: {self.get_precision(status_code)}'
                )
            elif not self.gps_healthy and old_healthy:
                self.get_logger().warn(
                    f'⚠️  GPS perdu - Status: {status_text} (code: {status_code})'
                )
            elif old_status != self.current_status:
                self.get_logger().info(
                    f'🔄 Changement qualité GPS - Status: {status_text} (code: {status_code}) - Précision: {self.get_precision(status_code)}'
                )
                
        # Publier l'état (toujours, même sans log)
        self.publish_health_state()
    
    def get_precision(self, status_code):
        """Retourne la précision en fonction du code status"""
        precision_map = {
            4: '1-3 cm',
            5: '20 cm - 1 m',
            2: '0.5-2 m',
            1: '2-5 m',
            0: 'AUCUNE',
            -1: 'AUCUNE'
        }
        return precision_map.get(status_code, 'INCONNUE')
            
    def check_gps_timeout(self):
        """Vérifie si le GPS n'a pas envoyé de données depuis trop longtemps"""
        now = self.get_clock().now()
        
        # Vérifier la source la plus récente
        gps_status_elapsed = (now - self.last_gps_status_time).nanoseconds / 1e9
        fix_elapsed = (now - self.last_fix_time).nanoseconds / 1e9
        
        # Si on utilise /gps/status en priorité, ne vérifier que lui
        if self.use_gps_status:
            elapsed = gps_status_elapsed
            source = '/gps/status'
        else:
            elapsed = min(gps_status_elapsed, fix_elapsed)
            source = 'GPS'
        
        if elapsed > self.timeout and self.gps_healthy:
            self.get_logger().warn(
                f'⏱️  GPS TIMEOUT - plus de données depuis {elapsed:.1f}s (source: {source})'
            )
            self.gps_healthy = False
            self.current_status = -1
            self.current_status_text = 'TIMEOUT'
            self.publish_health_state()
    
    def publish_health_state(self):
        """Publie l'état de santé GPS sur /gps/healthy"""
        if self.get_parameter('publish_health').value:
            msg = Bool()
            msg.data = self.gps_healthy
            self.health_pub.publish(msg)
            
    def enable_callback(self, request, response):
        """Service pour activer/désactiver manuellement"""
        self.gps_healthy = request.data
        self.publish_health_state()
        response.success = True
        response.message = f'GPS health set to {self.gps_healthy}'
        return response
        
    def get_status_text(self, status):
        """Convertit le code status GPS en texte lisible"""
        status_map = {
            -1: 'NO FIX',
            0: 'NO FIX',
            1: 'GPS FIX',
            2: 'DGPS FIX',
            4: 'RTK FIXED',
            5: 'RTK FLOAT'
        }
        return status_map.get(status, f'UNKNOWN ({status})')

def main(args=None):
    rclpy.init(args=args)
    node = GPSHealthMonitor()
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