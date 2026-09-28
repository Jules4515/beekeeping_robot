#!/usr/bin/env python3
# 
# GPS Mission node for ROS2
#
# This node loads GPS waypoints from a YAML trajectory, converts them into map
# poses, displays them in RViz, and sends them to Nav2 in Fly-By mode. It also
# pauses at TASK waypoints, publishes the current goal number for debug purposes, monitors GPS
# status, and stops the robot safely at the end of the mission.
# 

import sys

import yaml
import os
import math
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.executors import ExternalShutdownException
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from visualization_msgs.msg import Marker, MarkerArray
from robot_localization.srv import FromLL
from std_msgs.msg import String, Int32
from action_msgs.srv import CancelGoal

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class GpsMissionNode(Node):
    def __init__(self):
        super().__init__('gps_mission_node')
        
        # ROS parameter used to select the exact trajectory file.
        self.declare_parameter('trajectory_file', 'trajectory_20260709_113555.yaml')
        self.trajectory_file = self.get_parameter('trajectory_file').value

        # Pause duration for TASK waypoints.
        self.declare_parameter('task_wait_time', 5.0)
        self.task_wait_time = self.get_parameter('task_wait_time').value

        print("[INFO] To launch with a custom file: ros2 run bee_mobile gps_mission_node --ros-args -p trajectory_file:=my_file.yaml")
        
        self.navigator = BasicNavigator()
        self.gps_status = {'emoji': '🔴', 'text': 'WAITING...'}
        
        # Subscriptions and publishers.
        self.status_sub = self.create_subscription(
            String, '/gps/status', self.gps_status_callback, 10)
        
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/points_visuels', qos)
        self.goal_num_pub = self.create_publisher(Int32, '/goal_number', 10)
        
        self.from_ll_client = self.create_client(FromLL, '/fromLL')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

    def gps_status_callback(self, msg):
        data = msg.data
        
        # Check for the status within the complete message string.
        if 'RTK_FIXED' in data:
            self.gps_status = {'emoji': '🟢', 'text': data}
        elif 'RTK_FLOAT' in data:
            self.gps_status = {'emoji': '🔵', 'text': data}
        elif 'DGPS' in data:
            self.gps_status = {'emoji': '🟡', 'text': data}
        elif 'GPS_POOR' in data:
            self.gps_status = {'emoji': '🟠', 'text': data}
        else:
            self.gps_status = {'emoji': '🔴', 'text': data}

    def print_gps_banner(self):
        emoji = self.gps_status['emoji']
        text = self.gps_status['text']
        print("\n" + "="*50)
        print(f"  GPS STATUS: {emoji} {text}")
        print("="*50 + "\n")

    @staticmethod
    def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
        """Convert Euler angles in degrees to a 3D quaternion."""
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def execute_mission(self):
        """Execute the main mission workflow."""
        self.get_logger().info("Waiting for Nav2...")
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')
        
        self.get_logger().info("Waiting for GPS data...")
        timeout = time.time() + 5.0
        while self.gps_status['text'] == 'WAITING...' and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.print_gps_banner()
        if 'RTK_FIXED' not in self.gps_status['text'] and 'DGPS' not in self.gps_status['text'] and self.gps_status['text'] != 'WAITING...':
            self.get_logger().warn("No optimal correction (RTK_FIXED / DGPS) detected!")
            input("Press ENTER to continue anyway...")

        self.get_logger().info("Waiting for the GPS conversion service (/fromLL)...")
        while not self.from_ll_client.wait_for_service(timeout_sec=2.0):
            rclpy.spin_once(self, timeout_sec=0.1)

        # ============================================================
        # WAYPOINT LOADING AND CONVERSION
        # ============================================================
        ws_path = os.path.expanduser('~/dev/robot_ws/src/bee_mobile/trajectories')
        os.makedirs(ws_path, exist_ok=True)
        yaml_path = os.path.join(ws_path, self.trajectory_file)
        
        # Fall back if the requested file does not exist.
        if not os.path.exists(yaml_path):
            self.get_logger().error(f"File not found: {yaml_path}")
            self.get_logger().info("Fallback to default file: trajectory_DEFAULT.yaml")
            self.trajectory_file = 'trajectory_DEFAULT.yaml'
            yaml_path = os.path.join(ws_path, self.trajectory_file)
            
            # Safety check for the default file.
            if not os.path.exists(yaml_path):
                self.get_logger().error("Default file not found. Mission aborted.")
                return

        self.get_logger().info(f"Loading trajectory: {self.trajectory_file}")
        with open(yaml_path, 'r') as f:
            waypoints = yaml.safe_load(f).get('waypoints_GPS', [])

        map_poses = []
        marker_array = MarkerArray()

        self.get_logger().info("Generating spatial footprint (RViz markers)...")
        for i, wp in enumerate(waypoints):
            # Call the map conversion service.
            req = FromLL.Request()
            req.ll_point.latitude = wp['latitude']
            req.ll_point.longitude = wp['longitude']
            req.ll_point.altitude = wp['altitude']

            future = self.from_ll_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            map_point = future.result().map_point

            # Build the 6-DoF pose.
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position = map_point
            pose.pose.orientation = self.euler_to_quaternion(wp['roll'], wp['pitch'], wp['yaw'])
            map_poses.append(pose)

            # Marker 1: Blue cylinder (position reference).
            marker_cyl = Marker()
            marker_cyl.header = pose.header
            marker_cyl.ns = 'waypoints_position'
            marker_cyl.id = i * 2
            marker_cyl.type = Marker.CYLINDER
            marker_cyl.action = Marker.ADD
            marker_cyl.pose.position = pose.pose.position
            marker_cyl.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) # Cylinder lying flat.
            marker_cyl.scale.x = 0.3
            marker_cyl.scale.y = 0.3
            marker_cyl.scale.z = 0.1
            if 'TASK' in wp['name']:
                marker_cyl.color.r, marker_cyl.color.g, marker_cyl.color.b, marker_cyl.color.a = 1.0, 0.0, 1.0, 0.8
            else:
                marker_cyl.color.r, marker_cyl.color.g, marker_cyl.color.b, marker_cyl.color.a = 0.0, 0.5, 1.0, 0.8
            marker_array.markers.append(marker_cyl)

            # Marker 2: Red arrow (6-DoF orientation).
            marker_arr = Marker()
            marker_arr.header = pose.header
            marker_arr.ns = 'waypoints_orientation'
            marker_arr.id = i * 2 + 1
            marker_arr.type = Marker.ARROW
            marker_arr.action = Marker.ADD
            marker_arr.pose = pose.pose # Inherits the robot's absolute rotation.
            marker_arr.scale.x = 0.4  # Length.
            marker_arr.scale.y = 0.05 # Shaft thickness.
            marker_arr.scale.z = 0.05 # Head thickness.
            marker_arr.color.r, marker_arr.color.g, marker_arr.color.b, marker_arr.color.a = 1.0, 0.0, 0.0, 1.0
            marker_arr.color.r, marker_arr.color.g, marker_arr.color.b, marker_arr.color.a = 1.0, 0.0, 0.0, 1.0
            marker_array.markers.append(marker_arr)

        # Publish all shapes together.
        for i in range(3):  # Repeat publication to ensure RViz receives it.
            self.marker_pub.publish(marker_array)
        time.sleep(0.5)

        # ============================================================
        # FLY-BY EXECUTION MODE
        # ============================================================
        print("\n" + "="*50)
        print("MISSION START (FLY-BY MODE)")
        self.print_gps_banner()
        print("="*50)
        input(f"\n> {len(map_poses)} points loaded. Press ENTER to start the robot... (Ctrl+C to cancel)")

        # Fly-By validation radius in meters.
        # A larger radius makes the robot switch to the next point earlier.
        FLY_BY_RADIUS = 1.5

        for i in range(len(map_poses)):
            target_pose = map_poses[i]
            wp_name = waypoints[i]['name']
            is_last_point = (i == len(map_poses) - 1)
            
            # Semantic discriminator.
            is_task_wp = 'TASK' in wp_name
            
            goal_msg = Int32()
            goal_msg.data = i + 1
            self.goal_num_pub.publish(goal_msg)

            print(f"\n---> Heading to: [{wp_name}]")
            self.navigator.goToPose(target_pose)

            # =========================================================
            # RACE CONDITION MITIGATION
            # =========================================================
            if not is_last_point and i > 0:
                blind_timeout = time.time() + 1.0
                while time.time() < blind_timeout:
                    rclpy.spin_once(self, timeout_sec=0.05)

            # =========================================================
            # STANDARD MONITORING LOOP
            # =========================================================
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                
                if feedback and hasattr(feedback, 'distance_remaining'):
                    # Execute Fly-By ONLY when this is not a TASK waypoint.
                    if not is_last_point and not is_task_wp and feedback.distance_remaining < FLY_BY_RADIUS:
                        print(f"  [FLY-BY] {FLY_BY_RADIUS} m radius reached. Skipping!")
                        break

                rclpy.spin_once(self, timeout_sec=0.05)

            # =========================================================
            # STOP HANDLING (TASK & MISSION END)
            # =========================================================
            # If Fly-By was not used, Nav2 has completed the trajectory.
            if is_task_wp or is_last_point:
                result = self.navigator.getResult()
                
                if result == TaskResult.SUCCEEDED:
                    if is_last_point:
                        print("  [SUCCESS] Final destination reached.")
                        
                    if is_task_wp and not is_last_point:
                        print(f"  [TASK] Stop point reached. Starting the {self.task_wait_time}s pause.")
                        
                        # Failsafe: publish zero commands to guarantee motor lock.
                        stop_msg = Twist()
                        for _ in range(3):
                            self.cmd_vel_pub.publish(stop_msg)
                            time.sleep(0.1)
                            
                        # Non-blocking wait (keeps ROS 2 communication active).
                        wait_timeout = time.time() + self.task_wait_time
                        while time.time() < wait_timeout:
                            rclpy.spin_once(self, timeout_sec=0.1)
                            
                        print("  [TASK] Pause complete. Resuming motion.")
                        
                elif result == TaskResult.FAILED:
                    print(f"  [FAILURE] Target {wp_name} is unreachable.")

        print("\n" + "="*50)
        print("MISSION COMPLETE!")
        self.print_gps_banner()
        print("="*50)
        
        # Failsafe: publish zero commands to guarantee a physical stop.
        stop_msg = Twist()
        for _ in range(3):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = GpsMissionNode()
    
    try:
        node.execute_mission()
    except (KeyboardInterrupt, ExternalShutdownException):
        print(f"[INFO] [{node.get_name()}]: Interruption requested. Shutting down cleanly...")
        
        # Isolate cancellation attempts in case the ROS context is corrupted.
        try:
            if hasattr(node.navigator, 'nav_to_pose_client') and node.navigator.nav_to_pose_client.server_is_ready():
                if hasattr(node.navigator, 'goal_handle') and node.navigator.goal_handle is not None:
                    future = node.navigator.goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(node.navigator, future, timeout_sec=1.0)
                else:
                    cancel_client = node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
                    if cancel_client.wait_for_service(timeout_sec=0.5):
                        req = CancelGoal.Request()
                        future = cancel_client.call_async(req)
                        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        except Exception:
            pass # Ignore silently if the context is no longer valid.
            
    finally:
        # Final cleanup.
        try:
            node.navigator.destroy_node()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        
        # Force system exit with code 0 to prevent the
        # "Process exited with failure 1" log.
        sys.exit(0)

if __name__ == '__main__':
    main()