import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('bee_mobile')

    # --- Arguments ---
    sim_arg = DeclareLaunchArgument(
        'simulation',
        default_value='False',
        description='Set to True to enable SITL (Software-in-the-Loop) simulation bypasses and disable hardware'
    )
    is_sim = LaunchConfiguration('simulation')

    # --- URDF & Robot State (Always On) ---
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False, 'publish_frequency': 50.0,}]  # Increased from 50 to 100 Hz
    )

    # --- Hardware Nodes (Real World Only) ---
    micro_ros_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        arguments=['udp4', '--port', '8888'],
        # arguments=['udp4', '--port', '8888', '-v6']   # -v6 for verbose, optional
        condition=UnlessCondition(is_sim)
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'lidar.launch.py')),
        condition=UnlessCondition(is_sim)
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'camera.launch.py')),
        condition=UnlessCondition(is_sim)
    )

    aruco_tf_broadcaster_node = Node(
        package='bee_mobile',
        executable='aruco_tf_broadcaster',
        output='screen',
        condition=UnlessCondition(is_sim)
    )

    # compressed_aruco_node = Node(
    #     package='bee_mobile', 
    #     executable='compressed_aruco_node',
    #     name='aruco_node',
    #     namespace='camera',
    #     output='screen',
    #     parameters=[{
    #         'marker_size': 0.068,
    #         'image_topic': '/camera/image_raw/compressed',
    #         #'enable_debug': True  # Force la désactivation complète du traitement d'image inutile
    #     }]
    # )

    docking_controller_node = Node(
        package='bee_mobile',
        executable='docking_controller',
        output='screen',
        condition=UnlessCondition(is_sim)
    )

    # --- Simulation Nodes (SITL Only) ---
    sim_bridge_node = Node(
        package='bee_mobile',
        executable='sim_bridge',
        output='screen',
        condition=IfCondition(is_sim)
    )

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(is_sim)
    )

    # --- Core Control Nodes (Always On) ---
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen'
    )

    mux_joystick_node = Node(
        package='bee_mobile',
        executable='mux_joystick_node',
        output='screen'
    )

    swerve_kinematics_node = Node(
        package='bee_mobile',
        executable='swerve_kinematics',
        output='screen'
    )

    odometry_node = Node(
        package='bee_mobile',
        executable='odometry',
        output='screen'
    )
    
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'twist_mux_topics.yaml')]
    )

    return LaunchDescription([
        sim_arg,
        robot_state_publisher_node,
        #micro_ros_node,
        lidar_launch,
        camera_launch,
        #aruco_tf_broadcaster_node,
        sim_bridge_node,
        static_tf_map_odom,
        joy_node,
        mux_joystick_node,
        #swerve_kinematics_node,
        odometry_node,
        twist_mux_node,
        docking_controller_node,
    ])