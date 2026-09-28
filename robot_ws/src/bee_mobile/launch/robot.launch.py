import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('bee_mobile')

    # ========================================================================
    # 1. LAUNCH ARGUMENTS (Outdoor first, Swerve first)
    # ========================================================================
    
    # Declare environment argument (outdoor or indoor)
    environment_arg = DeclareLaunchArgument(
        'environment', 
        default_value='outdoor', 
        choices=['outdoor', 'indoor'],
        description='Navigation environment: outdoor or indoor'
    )

    # Declare kinematics argument (swerve or ackermann)
    kinematics_arg = DeclareLaunchArgument(
        'kinematics', 
        default_value='swerve', 
        choices=['swerve', 'ackermann'],
        description='Kinematics model: swerve or ackermann'
    )

    # Extract the configurations to use them in dynamic paths
    environment_mode = LaunchConfiguration('environment')
    kinematics_mode = LaunchConfiguration('kinematics')

    # ========================================================================
    # 2. DYNAMIC PATHS (PathJoinSubstitution)
    # ========================================================================
    
    # Dynamically build the Nav2 parameters path 
    # Example result: config/nav2_params_outdoor_swerve.yaml
    nav2_params_file = PathJoinSubstitution([
        pkg_share, 
        'config', 
        ['nav2_params_', environment_mode, '_', kinematics_mode, '.yaml']
    ])

    # Dynamically build the navigation launch file path
    # Example result: launch/navigation_outdoor.launch.py
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share, 
                'launch', 
                ['navigation_', environment_mode, '.launch.py']
            ])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file  # Passing the dynamically resolved YAML
        }.items()
    )

    # ========================================================================
    # 3. NODES DEFINITION
    # ========================================================================

    # --- URDF & Robot State (Always On) ---
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False, 'publish_frequency': 15.0}]
    )

    # --- Hardware Nodes (Real World Only) ---
    micro_ros_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        arguments=['udp4', '--port', '8888'],
        # arguments=['udp4', '--port', '8888', '-v6']   # -v6 for verbose, optional
    )

    unitree_lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar',
        parameters=[{
            'cloud_frame': 'unilidar_lidar',
            'imu_frame': 'unilidar_imu',
        }],
        remappings=[
            ('/tf', '/tf_unitree_ignored'),
            ('/tf_static', '/tf_static_unitree_ignored')
        ]
    )

    pointcloud_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/unilidar/cloud'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
            'min_height': 0.15,
            'max_height': 0.80,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.05,
            'range_min': 0.30,
            'range_max': 15.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }]
    )

    camera_info_yaml = os.path.join(pkg_share, 'config', 'camera_info.yaml')
    
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'framerate': 30.0,
            'pixel_format': 'mjpeg2rgb',
            'image_width': 640,
            'image_height': 480,
            'camera_frame_id': 'camera_link',
            'camera_info_url': 'file://' + camera_info_yaml,
            'exposure_auto': 1,       
            'exposure_absolute': 20,
            'gain': 10,
            'qos_reliability': 'best_effort',
            'qos_history': 'keep_last',
            'qos_depth': 1,
        }]
    )

    pid_tuner_node = Node(
        package='bee_mobile',
        executable='pid_tuner',
        output='screen',
        parameters=[{
            'interactive_mode': False
        }]
    )

    aruco_tag_detector_node = Node(
        package='bee_mobile', 
        executable='aruco_tag_detector',
        name='aruco_tag_detector',
        namespace='camera',
        output='screen',
        parameters=[{
            'marker_size': 0.068,
            'image_topic': '/camera/image_raw/compressed',
            'enable_debug': False,  
            'publish_pose_array': False
        }]
    )

    docking_controller_node = Node(
        package='bee_mobile',
        executable='docking_controller',
        output='screen',
    )

    # --- Core Control Nodes (Always On) ---
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen'
    )

    # Dynamic Executable: Resolves to 'joystick_swerve' or 'joystick_ackermann'
    joystick_node = Node(
        package='bee_mobile',
        executable=['joystick_', kinematics_mode],
        output='screen'
    )

    # Dynamic Executable: Resolves to 'kinematics_swerve' or 'kinematics_ackermann'
    kinematics_node = Node(
        package='bee_mobile',
        executable=['kinematics_', kinematics_mode],
        output='screen'
    )

    # Dynamic Executable: Resolves to 'odometry_swerve' or 'odometry_ackermann'
    odometry_node = Node(
        package='bee_mobile',
        executable=['odometry_', kinematics_mode],
        output='screen'
    )

    unitree_imu_hotfix_node = Node(
        package='bee_mobile',
        executable='unitree_imu_hotfix',
        output='screen'
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'twist_mux_topics.yaml')]
    )

    # ========================================================================
    # 4. RETURN LAUNCH DESCRIPTION
    # ========================================================================
    
    return LaunchDescription([
        # Arguments
        environment_arg,
        kinematics_arg,
        
        # Static Nodes
        robot_state_publisher_node,
        
        #micro_ros_node,
        unitree_lidar_node,
        pointcloud_to_scan_node,
        #camera_node,
        aruco_tag_detector_node,
        pid_tuner_node,
        
        # Control & Dynamic Nodes
        joy_node,
        joystick_node,
        kinematics_node,
        odometry_node,
        unitree_imu_hotfix_node,
        twist_mux_node,
        #docking_controller_node,

        # Included Launch Files
        navigation_launch,
    ])