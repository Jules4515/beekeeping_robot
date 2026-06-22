from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # NMEA Driver (GPS)
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_serial_driver',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',  # Verify this is correct
                'baud': 115200,
            }]
        ),
        
        # Heading Publisher
        Node(
            package='bee_mobile',
            executable='heading_publisher',
            name='heading_publisher',
            output='screen',
        ),
        
        # Robot Footprint Publisher
        Node(
            package='bee_mobile',
            executable='robot_footprint_publisher',
            name='robot_footprint_publisher',
            output='screen',
        ),
        
        # Initialize Origin Transform
        Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            output='screen',
            remappings=[
                ('fix', '/fix'),
            ],
            parameters=[{
                'local_xy_frame': 'map',
                'local_xy_origin': 'auto',
            }],
        ),
        
        # Static TF from map to base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        ),
        
        # # Mapviz
        # Node(
        #     package='mapviz',
        #     executable='mapviz',
        #     name='mapviz',
        #     output='screen',
        # ),
    ])