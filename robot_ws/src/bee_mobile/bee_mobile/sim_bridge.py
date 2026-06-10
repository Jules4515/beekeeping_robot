# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray

# class SimBridgeNode(Node):
#     """
#     Simulates perfect hardware by directly looping back motor commands 
#     into encoder feedback topics.
#     """
#     def __init__(self):
#         super().__init__('sim_bridge_node')

#         self.wheel_modules = ['front_left', 'front_right', 'rear_left', 'rear_right']
#         self.pubs = {}

#         for name in self.wheel_modules:
#             # Publisher for the fake encoder feedback
#             pub_topic = f'mobile/wheel_{name}/encoder_angle'
#             self.pubs[name] = self.create_publisher(Float64MultiArray, pub_topic, 10)

#             # Subscriber to the commands sent by swerve_kinematics_node
#             sub_topic = f'mobile/wheel_{name}/motor_speed'
#             self.create_subscription(
#                 Float64MultiArray, 
#                 sub_topic, 
#                 lambda msg, n=name: self.cmd_callback(msg, n), 
#                 10
#             )

#         self.get_logger().info("Sim Bridge started: Looping /motor_speed -> /encoder_angle")

#     def cmd_callback(self, msg, wheel_name):
#         # Pretend the motor instantly reached the target RPM and Angle
#         # and publish it back as if it came from the ESP32 encoders
#         self.pubs[wheel_name].publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = SimBridgeNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class SimBridgeNode(Node):
    """
    Simulates perfect hardware by maintaining the state of the motors
    and broadcasting encoder feedback at a strict, continuous frequency (50Hz).
    """
    def __init__(self):
        super().__init__('sim_bridge_node')

        self.wheel_modules = ['front_left', 'front_right', 'rear_left', 'rear_right']
        self.pubs = {}
        
        # Hardware state memory (RPM, Angle)
        self.hardware_state = {name: [0.0, 0.0] for name in self.wheel_modules}

        for name in self.wheel_modules:
            # Publisher for the fake encoder feedback
            pub_topic = f'mobile/wheel_{name}/encoder_angle'
            self.pubs[name] = self.create_publisher(Float64MultiArray, pub_topic, 10)

            # Subscriber to the commands sent by swerve_kinematics_node
            sub_topic = f'mobile/wheel_{name}/motor_speed'
            self.create_subscription(
                Float64MultiArray, 
                sub_topic, 
                lambda msg, n=name: self.cmd_callback(msg, n), 
                10
            )

        # Hardware simulation timer: Strictly 50Hz to match Kinematics Odometry
        self.timer = self.create_timer(0.02, self.publish_fake_encoders)
        
        self.get_logger().info("Sim Bridge started: Continuous 50Hz hardware simulation.")

    def cmd_callback(self, msg, wheel_name):
        """ Asynchronously updates the simulated hardware memory """
        if len(msg.data) >= 2:
            # Assume instant physical response for simulation purposes
            self.hardware_state[wheel_name] = [float(msg.data[0]), float(msg.data[1])]

    def publish_fake_encoders(self):
        """ Synchronously broadcasts encoder feedback at 50Hz """
        for name in self.wheel_modules:
            state = self.hardware_state[name]
            msg = Float64MultiArray(data=state)
            self.pubs[name].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()