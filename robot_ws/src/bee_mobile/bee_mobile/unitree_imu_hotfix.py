#!/usr/bin/env python3
# 
# Unitree IMU Hotfix node for ROS2
#
# This node corrects the Unitree LiDAR IMU coordinate mapping before republishing
# the data. It remaps and inverts the raw roll orientation and X-axis angular
# velocity into yaw and Z-axis angular velocity, while clearing the unused
# roll, pitch, X, and Y components.
# 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class UnitreeImuHotfix(Node):
    def __init__(self):
        super().__init__('unitree_imu_hotfix')
        
        self.sub = self.create_subscription(Imu, '/unilidar/imu', self.imu_callback, 10)
        self.pub = self.create_publisher(Imu, '/unilidar/imu_corrected', 10)
        
        self.get_logger().info(
            "\n"
            "============================================================\n"
            "UNITREE IMU HOTFIX ACTIVATED (FORWARD ALIGNED + INVERTED)\n"
            "============================================================\n"
            "Action:\n"
            "Intercepting at 250Hz. Roll is remapped to Yaw AND INVERTED.\n"
            "The raw X-axis angular velocity is remapped to Z-axis AND \n"
            "INVERTED. Roll, Pitch, wx, and wy are zeroed out.\n"
            "============================================================"
        )

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        return roll

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0.0] * 4
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy
        return q

    def imu_callback(self, msg):
        corrected_msg = Imu()
        corrected_msg.header = msg.header
        corrected_msg.header.frame_id = 'unilidar_imu'

        # 1. Extract the incorrect roll (X).
        old_roll = self.euler_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        # 2. Transfer and invert: roll -> yaw.
        new_yaw = -old_roll   # <-- ANGLE INVERSION
        new_roll = 0.0
        new_pitch = 0.0

        q = self.quaternion_from_euler(new_roll, new_pitch, new_yaw)
        corrected_msg.orientation.x = q[0]
        corrected_msg.orientation.y = q[1]
        corrected_msg.orientation.z = q[2]
        corrected_msg.orientation.w = q[3]
        corrected_msg.orientation_covariance = msg.orientation_covariance

        # 3. Transfer and invert angular velocity: wx -> wz.
        corrected_msg.angular_velocity.x = 0.0
        corrected_msg.angular_velocity.y = 0.0
        corrected_msg.angular_velocity.z = -msg.angular_velocity.x  # <-- VELOCITY INVERSION
        corrected_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        
        # 4. Pass linear acceleration through unchanged.
        corrected_msg.linear_acceleration = msg.linear_acceleration
        corrected_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(corrected_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UnitreeImuHotfix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()