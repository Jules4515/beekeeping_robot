#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class PIDTuner(Node):
    """
    Sends identical PID coefficients to all four wheel modules.
    Each module expects 8 values: [Kp_drive, Ki_drive, Kd_drive, Offset_drive,
                                    Kp_steer, Ki_steer, Kd_steer, Offset_steer]
    """
    def __init__(self):
        super().__init__('pid_tuner')

        # Control variable for interactive input
        self.declare_parameter('interactive_mode', True)
        self.interactive_mode = self.get_parameter('interactive_mode').value

        # Wheel modules
        self.wheel_modules = ['front_left', 'front_right', 'rear_left', 'rear_right']

        # Publishers for each wheel
        self.pid_pubs = {}
        for name in self.wheel_modules:
            topic = f'mobile/wheel_{name}/pid_params'
            self.pid_pubs[name] = self.create_publisher(Float64MultiArray, topic, 10)

        # Default PID values (tuned to reduce pivot‑mode oscillation)
        # [Kp_d, Ki_d, Kd_d, Off_d,   Kp_s, Ki_s, Kd_s, Off_s]
        self.default_pid = [
            20.0, 3.2, 0.2, 0.0,    # Drive: Kp, Ki, Kd, Offset (stable)
            20.0, 0.5, 2.0, 130.0  #4.0, 0.5, 0.5, 120.0    # Steer: Kp lower (0.2) to reduce 45° oscillation
        ]
        
        # Publish hardcoded values once at startup
        self.publish_to_all(self.default_pid)

        if self.interactive_mode:
            self.get_logger().info('PID Tuner ready – sending same 8 coefficients to all wheels.')
            self.get_logger().info('Enter 8 numbers: Kp_d Ki_d Kd_d Off_d   Kp_s Ki_s Kd_s Off_s')
            self.get_logger().info('Example to reduce steering oscillation: 0.5 0 0 350   0.15 0 0.02 0')
        else:
            self.get_logger().info('Interactive mode disabled. Applied hardcoded PID values.')

    def publish_to_all(self, coeffs):
        """Publish the same 8‑element array to every wheel module."""
        if len(coeffs) != 8:
            self.get_logger().error('Must provide exactly 8 coefficients')
            return
        msg = Float64MultiArray()
        msg.data = coeffs
        for name, pub in self.pid_pubs.items():
            pub.publish(msg)
        self.get_logger().info(
            f'Published to all wheels: Drive(Kp={coeffs[0]:.3f}, Ki={coeffs[1]:.3f}, '
            f'Kd={coeffs[2]:.3f}, Off={coeffs[3]:.1f}) | '
            f'Steer(Kp={coeffs[4]:.3f}, Ki={coeffs[5]:.3f}, Kd={coeffs[6]:.3f}, Off={coeffs[7]:.1f})'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PIDTuner()

    try:
        if node.interactive_mode:
            # Interactive loop: wait for user input
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                user_input = input("\nEnter 8 PID values (or 'q' to quit): ").strip()
                if user_input.lower() == 'q':
                    break
                parts = user_input.split()
                if len(parts) == 8:
                    try:
                        new_pid = [float(x) for x in parts]
                        node.publish_to_all(new_pid)
                    except ValueError:
                        node.get_logger().error('Invalid numbers – use floats separated by spaces.')
                else:
                    node.get_logger().error('Please enter exactly 8 numbers.')
        else:
            # Static mode: just keep the node alive so subscribers can receive the message
            rclpy.spin(node)
            
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()