from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Path to the camera calibration file (K, P, R matrices)
    config = os.path.join(get_package_share_directory('bee_mobile'), 'config', 'camera_info.yaml')

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='camera', # Topics published under /camera
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb', # Captures MJPEG, converts to RGB8 for RViz
                'image_width': 640,
                'image_height': 480,
                'camera_frame_id': 'camera_frame', # TF frame, to be added to URDF
                # 'io_method': 'mmap', # Memory mapping can be more efficient
                'camera_info_url': 'file://' + config,
            }]
        ),
    ])