#!/usr/bin/env python3
# 
# Docking TF Math Tester
#
# !! Work in progress !!, file used by the docking controller and docking test nodes.
#
# This node inspects the robot odometry and ArUco tag transforms, extracts the
# tag's projected surface normal, and reports the geometric heading correction
# required to align the robot with the docking target (docking controller logic).
# 

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class DockingTfMathTester(Node):
    def __init__(self):
        super().__init__('docking_tf_math_tester')
        
        # Frames to observe
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        self.aruco_frame = 'aruco_marker_91'
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Slow test loop (2 Hz) to allow time to read the terminal output.
        self.timer = self.create_timer(0.5, self.diagnostic_loop)
        
        self.get_logger().info("TF MATH TESTER [ACTIVE] - Waiting for transforms...")

    def normalize_angle(self, angle):
        """Keep the angle strictly between -PI and +PI."""
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def get_transform(self, parent, child):
        """Test 2D odometry extraction (yaw)."""
        try:
            trans = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.05))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            
            # Convert quaternion to yaw (rotation around Z).
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return [x, y, yaw]
        except TransformException:
            return None

    def get_tag_quaternion(self, parent, child):
        """Test raw 3D extraction of the ArUco tag."""
        try:
            trans = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.05))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            q = trans.transform.rotation
            return [x, y, z, q.x, q.y, q.z, q.w]
        except TransformException:
            return None

    def diagnostic_loop(self):
        # 1. Measure 2D odometry.
        pose_robot = self.get_transform(self.odom_frame, self.base_frame)
        if not pose_robot:
            return  # Wait silently until odometry is available.
            
        rx, ry, ryaw = pose_robot

        # 2. Measure the tag in 3D relative to the robot frame.
        pose_tag = self.get_tag_quaternion(self.base_frame, self.aruco_frame)
        
        if pose_tag:
            tx, ty, tz, qx, qy, qz, qw = pose_tag
            
            # --- SURFACE NORMAL VECTOR MATH ---
            # Extract the X and Y components of the tag's outward Z vector.
            vec_z_x = 2.0 * (qx * qz + qw * qy)
            vec_z_y = 2.0 * (qy * qz - qw * qx)
            
            # Angle of the hive normal vector relative to the robot chassis.
            tag_normal_angle = math.atan2(vec_z_y, vec_z_x)
            
            # Target geometric angle (anti-parallel).
            dtheta_target = self.normalize_angle(tag_normal_angle + math.pi)
            
            # Diagnostic output.
            self.get_logger().info(
                f"\n=== GEOMETRIC DIAGNOSTIC (2 Hz) ===\n"
                f"[ROBOT IN ODOM]\n"
                f"  X: {rx:>6.3f}m | Y: {ry:>6.3f}m | Yaw: {math.degrees(ryaw):>6.1f}°\n"
                f"[RAW 3D TAG IN BASE_LINK]\n"
                f"  X: {tx:>6.3f}m | Y: {ty:>6.3f}m | Z (Height): {tz:>6.3f}m\n"
                f"  Qx: {qx:>5.2f} | Qy: {qy:>5.2f} | Qz: {qz:>5.2f} | Qw: {qw:>5.2f}\n"
                f"[NORMAL EXTRACTION (2D PROJECTION)]\n"
                f"  Hive Normal Vector: X_proj={vec_z_x:>5.2f}, Y_proj={vec_z_y:>5.2f}\n"
                f"  Hive Normal Angle: {math.degrees(tag_normal_angle):>6.1f}°\n"
                f"[ROBOT ALIGNMENT ERROR]\n"
                f"  Required Wz correction: {math.degrees(dtheta_target):>6.1f}°\n"
                f"====================================="
            )
        else:
            # Alternative output when the tag is not detected.
            self.get_logger().info(
                f"\n=== GEOMETRIC DIAGNOSTIC (2 Hz) ===\n"
                f"[ROBOT IN ODOM]\n"
                f"  X: {rx:>6.3f}m | Y: {ry:>6.3f}m | Yaw: {math.degrees(ryaw):>6.1f}°\n"
                f"[ARUCO TAG]\n"
                f"  NOT DETECTED / OUT OF VIEW\n"
                f"====================================="
            )

def main(args=None):
    rclpy.init(args=args)
    node = DockingTfMathTester()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print(f"[INFO] [{node.get_name()}]: Shutdown requested by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()