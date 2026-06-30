#!/usr/bin/env python3
import os
import json
import math
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from ament_index_python.packages import get_package_share_directory

def main(args=None):
    rclpy.init(args=args)

    # Temporary node to fetch the filename parameter
    param_node = rclpy.create_node('mission_param_loader')
    param_node.declare_parameter('mission_file', '')
    file_name = param_node.get_parameter('mission_file').value
    param_node.destroy_node()

    if not file_name:
        print("[ERROR] Missing 'mission_file' parameter.")
        print("Usage: ros2 run bee_mobile waypoint_commander --ros-args -p mission_file:=<filename.json>")
        print("Note: The JSON file MUST be located in the 'config' directory of the 'bee_mobile' package.")
        rclpy.shutdown()
        return

    # Dynamically resolve the absolute path to the config directory
    try:
        pkg_share = get_package_share_directory('bee_mobile')
        config_dir = os.path.join(pkg_share, 'config')
        json_path = os.path.join(config_dir, file_name)
        print(f"[INFO] Looking for mission file in: {config_dir}")
    except Exception as e:
        print(f"[ERROR] Package 'bee_mobile' not found: {e}")
        rclpy.shutdown()
        return

    # Load JSON file with specific error handling for missing files
    try:
        with open(json_path, 'r') as file:
            mission_data = json.load(file)
    except FileNotFoundError:
        print(f"[ERROR] File '{file_name}' not found.")
        print(f"Make sure '{file_name}' is inside 'bee_mobile/config/' and you have rebuilt the workspace.")
        rclpy.shutdown()
        return
    except Exception as e:
        print(f"[ERROR] Failed to read JSON file at '{json_path}': {e}")
        rclpy.shutdown()
        return

    # Initialize Nav2 Action Client
    navigator = BasicNavigator()
    
    # Bypass the AMCL health check in SITL since the Lidar is offline
    navigator.waitUntilNav2Active(localizer='bt_navigator')
    
    # Parse and convert to PoseStamped
    waypoints = []
    for wp in mission_data.get('waypoints', []):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        
        pose.pose.position.x = float(wp['x'])
        pose.pose.position.y = float(wp['y'])
        
        yaw = float(wp['yaw'])
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        
        waypoints.append(pose)

    if not waypoints:
        print("[WARN] Mission file contains no waypoints. Aborting.")
        navigator.lifecycleShutdown()
        rclpy.shutdown()
        return

    print(f"[INFO] Executing mission: {mission_data.get('mission_id', 'Unknown')} ({len(waypoints)} waypoints)")
    
    # Send sequence to Action Server
    navigator.followWaypoints(waypoints)

    # Block until the task is finished
    while not navigator.isTaskComplete():
        pass

    # Clean shutdown
    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()