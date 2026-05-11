import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('bee_mobile')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # Process xacro to URDF using the xacro command line tool
    #robot_description_raw = xacro.process_file(xacro_file).toxml()
    robot_description_raw = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    wheel_state_converter = Node(
        package='bee_mobile',
        executable='wheel_state_converter',
        name='wheel_state_converter',
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['/wheel_joint_states']}]
    )

    # Optional: static transform from odom to base_footprint (so you can keep fixed frame = odom)
    """ static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    ) """

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
        wheel_state_converter,
        joint_state_publisher,
        # static_tf,
        rviz2,
    ])