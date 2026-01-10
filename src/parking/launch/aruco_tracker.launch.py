from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            name='aruco_tracker',
            output='screen',
            parameters=[{
                # --- 1. TOPIC CONFIGURATION ---
                # We listen to the clean, rectified images from your camera node
                'cam_base_topic': '/camera/image_rect',
                'image_topic': '/camera/image_rect',
                'camera_info_topic': '/camera/camera_info',
                'image_is_rectified': True,
                # --- 2. FRAME CONFIGURATION ---
                # Must match 'self.frame_id' in your python script
                'camera_frame': 'camera_optical_frame',
                'marker_frame_prefix': 'marker',
                'publish_tf': True,


                'marker_dict': '6X6_50',
                'marker_size': 0.094,
                # Override for the one distinct marker (Example: ID 3 is 10cm)
                'marker_id_to_size': ["0:0.094", "1:0.094", "2:0.094", "3:0.037"], 
            }]
        )
    ])