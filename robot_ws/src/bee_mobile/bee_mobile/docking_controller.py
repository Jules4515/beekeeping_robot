#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class RucheDockingManager(Node):
    # State Machine Constants
    STATE_WAITING_NAV2 = 0
    STATE_SEARCHING    = 1
    STATE_PRE_ALIGNING = 2
    STATE_ALIGNING     = 3
    STATE_DOCKED       = 4

    def __init__(self):
        super().__init__('ruche_docking_manager')

        # --- Frames Configuration ---
        self.declare_parameter('target_frame', 'aruco_marker_91')
        self.declare_parameter('base_frame', 'base_link')
        
        # --- Velocity Profile Parameters (LOGICAL SPEEDS) ---
        self.declare_parameter('v_max_docking', 0.35)     # Absolute max approach speed
        self.declare_parameter('braking_distance', 1.0)   # Distance (m) where deceleration starts
        
        # --- Proportional Gains ---
        self.declare_parameter('kp_y', 1.5)
        self.declare_parameter('kp_yaw', 1.2)
        
        # --- Tolerances ---
        self.declare_parameter('tol_y', 0.05)             # 5cm max lateral error for pre-alignment
        self.declare_parameter('tol_yaw', 0.05)           # ~2.8 degrees max angular error
        self.declare_parameter('tol_x_docked', 0.25)      # Target stopping distance from the hive
        self.declare_parameter('hysteresis_factor', 2.0)  # Multiplier to avoid state chattering
        self.declare_parameter('timeout_tf_lost', 1.0)    # Seconds before triggering safety halt on vision loss

        # Retrieve parameters
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.v_max_docking = self.get_parameter('v_max_docking').get_parameter_value().double_value
        self.braking_distance = self.get_parameter('braking_distance').get_parameter_value().double_value
        
        self.kp_y = self.get_parameter('kp_y').get_parameter_value().double_value
        self.kp_yaw = self.get_parameter('kp_yaw').get_parameter_value().double_value
        self.tol_y = self.get_parameter('tol_y').get_parameter_value().double_value
        self.tol_yaw = self.get_parameter('tol_yaw').get_parameter_value().double_value
        self.tol_x_docked = self.get_parameter('tol_x_docked').get_parameter_value().double_value
        self.hysteresis = self.get_parameter('hysteresis_factor').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout_tf_lost').get_parameter_value().double_value

        self.current_state = self.STATE_WAITING_NAV2
        self.last_tf_time = self.get_clock().now()

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Interfaces
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10) # Handled downstream by swerve_kinematics
        
        self.trigger_sub = self.create_subscription(
            Bool, 
            '/docking/enable', 
            self.trigger_callback, 
            10
        )

        # 20Hz Control Loop
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Ruche Docking Manager Initialized. Waiting for Nav2...")

    def trigger_callback(self, msg):
        if msg.data and self.current_state == self.STATE_WAITING_NAV2:
            self.get_logger().info("Nav2 Goal Succeeded. Searching for Hive Tag...")
            self.current_state = self.STATE_SEARCHING

    def control_loop(self):
        if self.current_state in [self.STATE_WAITING_NAV2, self.STATE_DOCKED]:
            return

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.base_frame, self.target_frame, now)
            self.last_tf_time = self.get_clock().now()
            
            if self.current_state == self.STATE_SEARCHING:
                self.get_logger().info("Target Locked. Initiating Pre-Alignment (Crab).")
                self.current_state = self.STATE_PRE_ALIGNING

        except TransformException:
            time_since_last_tf = (self.get_clock().now() - self.last_tf_time).nanoseconds / 1e9
            if time_since_last_tf > self.timeout and self.current_state != self.STATE_SEARCHING:
                self.get_logger().error("Tag lost for > 1.0s. Emergency Halt.")
                self.halt_robot()
                self.current_state = self.STATE_SEARCHING
            return

        self.execute_kinematics(trans)

    def calculate_smooth_approach_speed(self, current_distance):
        """
        Calculates a C1 continuous velocity slope based on distance.
        Uses a square root profile to ensure constant deceleration during approach.
        """
        error_x = current_distance - self.tol_x_docked

        # Goal reached
        if error_x <= 0.0:
            return 0.0

        # Beyond braking distance: Cruising speed
        if error_x >= self.braking_distance:
            return self.v_max_docking
            
        # Inside braking zone: Smooth concave deceleration (Root curve)
        target_vx = self.v_max_docking * math.sqrt(error_x / self.braking_distance)
        
        return max(min(target_vx, self.v_max_docking), 0.0)

    def execute_kinematics(self, trans):
        msg = Twist()
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        
        # Quaternion to Yaw conversion (REP-103)
        q = trans.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Actual distance to the hive (Euclidean)
        distance = math.sqrt(x**2 + y**2)

        # STATE 2: PRE-ALIGNMENT (Crab & Pivot Only)
        if self.current_state == self.STATE_PRE_ALIGNING:
            if abs(y) <= self.tol_y and abs(yaw) <= self.tol_yaw:
                self.get_logger().info("Pre-alignment complete. Initiating Final Approach.")
                self.current_state = self.STATE_ALIGNING
                return
                
            msg.linear.x = 0.0
            # Send pure logical speeds. The swerve_kinematics node will handle hardware deadbands.
            msg.linear.y = float(self.kp_y * y)
            msg.angular.z = float(self.kp_yaw * yaw)
            
            self.cmd_pub.publish(msg)

        # STATE 3: FINAL APPROACH
        elif self.current_state == self.STATE_ALIGNING:
            
            if distance <= self.tol_x_docked:
                self.get_logger().info("Docking Complete. Ready for Manipulation.")
                self.halt_robot()
                self.current_state = self.STATE_DOCKED
                return

            # Hysteresis safety net
            if abs(y) > (self.tol_y * self.hysteresis) or abs(yaw) > (self.tol_yaw * self.hysteresis):
                self.get_logger().warn("Misalignment during approach. Reverting to Crab.")
                self.halt_robot()
                self.current_state = self.STATE_PRE_ALIGNING
                return

            # Apply the smooth C1 distance-based velocity slope
            msg.linear.x = self.calculate_smooth_approach_speed(distance)
            
            # Continuous micro-corrections
            msg.linear.y = float(self.kp_y * y)
            msg.angular.z = float(self.kp_yaw * yaw)
            
            self.cmd_pub.publish(msg)

    def halt_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RucheDockingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.halt_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()