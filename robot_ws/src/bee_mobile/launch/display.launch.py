import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('bee_mobile')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # Process xacro to URDF using the xacro command line tool
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # Joint state publisher (publishes zero angles for all joints)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['joint_states'], 'use_sim_time': False}]
    )

    # Optional: static transform from odom to base_footprint (so you can keep fixed frame = odom)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )

    # RViz2 node (optional config)
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'robot.rviz')] if os.path.exists(os.path.join(pkg_share, 'config', 'robot.rviz')) else []
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        static_tf,
        rviz2,
    ])