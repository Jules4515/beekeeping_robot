from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # USB camera node (publishes raw YUYV)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                'image_width': 640,
                'image_height': 480,
                'camera_frame_id': 'camera_frame',
            }]
        ),
    
        # YUYV to RGB converter node
        Node(
            package='bee_mobile',
            executable='yuyv_to_rgb',
            name='yuyv_to_rgb',
            output='screen'
        ),
    ])



# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         # 1. USB camera node (publishes raw YUYV or MJPEG)
#         Node(
#             package='usb_cam',
#             executable='usb_cam_node_exe',
#             name='usb_cam',
#             namespace='camera',
#             output='screen',
#             parameters=[{
#                 'video_device': '/dev/video0',
#                 'framerate': 30.0,
#                 'pixel_format': 'yuyv',  # or 'mjpeg' – both will be converted
#                 'image_width': 640,
#                 'image_height': 480,
#                 'camera_frame_id': 'camera_frame',
#             }]
#         ),

#         # 2. Image processing node: converts YUYV -> RGB8
#         Node(
#             package='image_proc',
#             executable='image_proc',
#             name='image_proc',
#             namespace='camera/usb_cam',
#             output='screen',
#             parameters=[{'output_encoding': 'rgb8'}],
#             remappings=[
#                 ('image_raw', 'image_raw'),      # input from usb_cam
#                 ('image_rect_color', 'image_rect_color')  # output RGB
#             ]
#         ),





        # 3. (Optional) If you want to view the RGB image directly:
        # Node(
        #     package='rqt_image_view',
        #     executable='rqt_image_view',
        #     name='rqt_image_view',
        #     output='screen',
        #     arguments=['/camera/usb_cam/image_rect_color']
        # ),
    #])


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='camera', # Isolates camera topics under /camera
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'framerate': 30.0,
                'pixel_format': 'yuyv', # 'mjpeg' is another common option
                'image_width': 640,
                'image_height': 480,
                'camera_frame_id': 'camera_frame', # TF frame, to be added to URDF
                # 'io_method': 'mmap', # Memory mapping can be more efficient
            }]
        ),
    ])