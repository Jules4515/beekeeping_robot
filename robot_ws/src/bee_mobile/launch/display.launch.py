import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('bee_mobile')

    # --- State Converters ---
    wheel_state_converter = Node(
        package='bee_mobile',
        executable='wheel_state_converter',
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['/wheel_joint_states'], 'use_sim_time': False, 'rate': 50.0}]
    )

    # --- Visualization ---
    rviz_config = os.path.join(pkg_share, 'config', 'robot.rviz')
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        # NOTE: robot_state_publisher is exclusively launched in robot.launch.py 
        # to prevent TF tree conflicts. Do not instantiate it here.
        wheel_state_converter,
        joint_state_publisher,
        rviz2
    ])