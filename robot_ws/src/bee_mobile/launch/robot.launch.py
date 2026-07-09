import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('bee_mobile')

    # --- URDF & Robot State (Always On) ---
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Navigation launch file path
    navigation_launch_path = os.path.join(pkg_share, 'launch', 'navigation_outdoor.launch.py')
    #navigation_launch_path = os.path.join(pkg_share, 'launch', 'navigation_indoor.launch.py')

    # Caméra
    camera_info_yaml = os.path.join(pkg_share, 'config', 'camera_info.yaml')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False, 'publish_frequency': 15.0,}]
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
            'exposure_auto': 1,       # 1 correspond souvent à un mode manuel ou priorité vitesse selon le pilote
            'exposure_absolute': 20,
            'gain': 10,
            'qos_reliability': 'best_effort',
            'qos_history': 'keep_last',
            'qos_depth': 1,
        }]
    )

    pid_tuner = Node(
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
            'enable_debug': False,  # Force la désactivation complète du traitement d'image inutile
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

    mux_joystick_node = Node(
        package='bee_mobile',
        executable='mux_joystick',
        output='screen'
    )

    swerve_kinematics_node = Node(
        package='bee_mobile',
        executable='new_swerve_kinematics',
        output='screen'
    )

    odometry_node = Node(
        package='bee_mobile',
        executable='odometry',
        output='screen'
    )

    unitree_imu_hotfix = Node(
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

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        # Prevents parent-child namespace collisions by explicitly passing arguments if required
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    return LaunchDescription([
        robot_state_publisher_node,
        #micro_ros_node,
        unitree_lidar_node,
        pointcloud_to_scan_node,
        #camera_node,
        aruco_tag_detector_node,
        pid_tuner,
        joy_node,
        mux_joystick_node,
        #swerve_kinematics_node,
        odometry_node,
        unitree_imu_hotfix,
        twist_mux_node,
        #docking_controller_node,
        navigation_launch,
    ])