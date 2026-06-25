#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import threading

class PIDTuner(Node):
    """
    Sends identical PID coefficients to wheel modules ONLY upon user input 
    or when a new microcontroller connects to the network (Topology Event).
    """
    def __init__(self):
        super().__init__('pid_tuner')

        self.declare_parameter('interactive_mode', True)
        self.interactive_mode = self.get_parameter('interactive_mode').value

        self.wheel_modules = ['front_left', 'front_right', 'rear_left', 'rear_right']
        self.pid_pubs = {}

        # Standard Volatile QoS is used. The node relies on active DDS graph monitoring instead.
        for name in self.wheel_modules:
            topic = f'mobile/wheel_{name}/pid_params'
            self.pid_pubs[name] = self.create_publisher(Float64MultiArray, topic, 10)

        # Default PID values
        # [Kp_d, Ki_d, Kd_d, Off_d, Kp_s, Ki_s, Kd_s, Off_s]
        self.active_pid = [20.0, 8.0, 4.0, 30.0, 30.0, 2.0, 1.0, 130.0] 

        self.data_lock = threading.Lock()

        # Topology tracking dictionary to detect late-joiners
        self.last_sub_count = {name: 0 for name in self.wheel_modules}
        
        # WHY: Monitor the DDS network at 10Hz. If a module's subscription count increases, 
        # it just connected/rebooted. We trigger a targeted publish.
        self.topology_timer = self.create_timer(0.1, self.check_topology_callback)

        if self.interactive_mode:
            self.get_logger().info('Event-Driven PID Tuner ready.')
            self.get_logger().info('Monitoring network for new MCU connections...')
        else:
            self.get_logger().info('Interactive mode disabled. Running in topology monitor mode.')

    def check_topology_callback(self):
        """Monitors DDS graph changes to send data only when a module connects."""
        trigger_publish = False
        
        for name, pub in self.pid_pubs.items():
            current_count = pub.get_subscription_count()
            
            # Positive edge detection: A microcontroller has joined the topic
            if current_count > self.last_sub_count[name]:
                self.get_logger().info(f'[{name}] Connection detected. Syncing PID parameters.')
                trigger_publish = True
                
            # Update state (handles disconnects gracefully without triggering a publish)
            self.last_sub_count[name] = current_count

        if trigger_publish:
            with self.data_lock:
                coeffs = list(self.active_pid)
            self.publish_to_all(coeffs)

    def update_pid_values(self, new_coeffs):
        """Thread-safe update triggered by user input."""
        with self.data_lock:
            self.active_pid = new_coeffs
        self.publish_to_all(new_coeffs)

    def publish_to_all(self, coeffs):
        if len(coeffs) != 8:
            self.get_logger().error('Must provide exactly 8 coefficients')
            return
            
        msg = Float64MultiArray(data=coeffs)
        for name, pub in self.pid_pubs.items():
            pub.publish(msg)

def input_thread_worker(node):
    while rclpy.ok():
        try:
            user_input = input("\nEnter 8 PID values (or 'q' to quit):\n").strip()
            if user_input.lower() == 'q':
                rclpy.shutdown()
                break
            
            parts = user_input.split()
            if len(parts) == 8:
                try:
                    new_pid = [float(x) for x in parts]
                    node.update_pid_values(new_pid)
                    node.get_logger().info(f'Manual override: PID updated to {new_pid}')
                except ValueError:
                    node.get_logger().error('Invalid numbers. Use floats.')
            else:
                node.get_logger().error('Please enter exactly 8 numbers.')
        except (EOFError, KeyboardInterrupt):
            break

def main(args=None):
    rclpy.init(args=args)
    node = PIDTuner()

    if node.interactive_mode:
        input_thread = threading.Thread(target=input_thread_worker, args=(node,), daemon=True)
        input_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()