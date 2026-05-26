import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ros_ws.src.bee_robot.bee_robot import four_ws_controller, joystick_node



def generate_launch_description():
    # Path to the scripts folder (not strictly needed, we'll use console scripts)
    pkg_share = get_package_share_directory('bee_mobile')

    # Include the lidar launch file
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'lidar.launch.py')
        )
    )

    # 1. micro-ROS Agent (UDP, port 8888)
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        # arguments=['udp4', '--port', '8888', '-v6']   # -v6 for verbose, optional
        arguments=['udp4', '--port', '8888']
    )

    # 2. joy_node (from the joy package)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # 3. joystick node (console script)
    joystick_node = Node(
        package='bee_mobile',
        executable='joystick_node',
        name='joystick_node',
        output='screen'
    )

    # 4. the four ws controller
    four_ws_controller = Node(
        package='bee_mobile',
        executable='four_ws_controller',
        name='four_ws_controller',
        output='screen'
    )

    # swerve_kinematics_node
    swerve_kinematics_node = Node(
        package='bee_mobile',
        executable='swerve_kinematics_node',
        name='swerve_kinematics_node',
        output='screen'
    )

    # 5. pid_tuner – optional
    pid_tuner = Node(
        package='bee_mobile',
        executable='pid_tuner',
        name='pid_tuner',
        output='screen'
    )

    # Include the SLAM launch file
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam.launch.py')
        )
    )

    # Inside generate_launch_description(), add:
    camera = IncludeLaunchDescription(
     PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bee_mobile'), 'launch', 'camera.launch.py')
        ) 
    )

    odometry = Node(
        package='bee_mobile',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
    )

    # Temporary node for nav2 loopback
    """ static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    ) """

    return LaunchDescription([
        # micro_ros_agent,
        joy_node,           # raw joystick driver
        #joystick_node,      # translates /joy → /cmd_vel + /mode_select
        #four_ws_controller, # kinematics + mode‑specific PID
        swerve_kinematics_node,
        lidar,
        # slam,
        camera,
        # odometry,
        # pid_tuner,          # remove this line if you prefer to start it manually
    ])

    # nav2 loopback
    """ return LaunchDescription([
        static_tf_map_odom
    ]) """