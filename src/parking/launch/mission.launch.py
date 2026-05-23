import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import subprocess, psutil

##Bash commands
##colcon build --symlink-install --parallel-workers 2
##source install/setup.bash
##ros2 launch parking mission.launch.py
##ros2 launch parking mission.launch.py 2>&1 | grep --line-buffered -v "Frame drop"
##ros2 topic pub --once /start_mission std_msgs/msg/Bool "{data: true}"

def pin_processes(context, *args, **kwargs):
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            cmd = ' '.join(proc.info['cmdline'] or [])
            pid = str(proc.pid)

            # Priority 1 — Odometry: VIP core, highest OS priority
            # rf2o and EKF must never be preempted by planning/vision work
            if 'rf2o' in cmd or 'ekf_node' in cmd:
                subprocess.run(['taskset', '-cp', '2', pid])
                subprocess.run(['renice', '-n', '-10', '-p', pid])

            # Priority 2 — MPPI local planner: dedicated cores for multithreading
            elif 'controller_server' in cmd:
                subprocess.run(['taskset', '-cp', '3,4,5', pid])

            # Priority 3 — Vision pipeline: GPU-backed Isaac ROS container,
            # box estimator, and AprilTag tracker share the same core pair
            elif ('component_container_mt' in cmd
                  or 'box_estimator' in cmd
                  or 'apriltag_tracker' in cmd):
                subprocess.run(['taskset', '-cp', '4,5', pid])

            # Priority 4 — Background: global planner and SLAM are latency-tolerant
            elif 'planner_server' in cmd or 'slam_toolbox' in cmd:
                subprocess.run(['taskset', '-cp', '0,1', pid])

        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return []


def generate_launch_description():
    # --- PATHS ---
    lidar_pkg = get_package_share_directory('ldlidar_stl_ros2')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    ##slam_pkg = get_package_share_directory('slam_toolbox')
    my_pkg = get_package_share_directory('parking')

    # A. Launch Sensors
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_pkg, 'launch', 'ld19.launch.py'))
    )
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(my_pkg, 'launch', 'camera.launch.py'))
    )

    camera_node = ComposableNode(
        name='argus_camera',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        parameters=[{
            'camera_id': 0,
            'camera_mode': 2,
            'fps': 15,
            'camera_info_url': 'file://' + os.path.join(my_pkg, 'config', 'camera_info.yaml'),
            'output_encoding': 'mono8'
        }],
        remappings=[
            ('left/image_raw', '/image_raw'),
            ('left/camera_info', '/camera_info')
        ]
    )

    rectify_node = ComposableNode(
        name='image_rectify',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{
            'output_width': 820,
            'output_height': 616,
        }],
        remappings=[
            ('image_raw', '/image_raw'),
            ('camera_info', '/camera_info'),
            ('image_rect', '/image_rect'),
            ('camera_info_rect', '/camera_info_rect')
        ]
    )

    # B. TF: base_link -> LiDAR
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=['--x', '0.08', '--y', '0', '--z', '0',
                   '--yaw', '1.5708', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'base_laser']
    )

    lidar_ghost_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.08', '--y', '0', '--z', '0',
                   '--yaw', '-1.5708', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'base_laser_nav']
    )

    # C. TF: base_link -> Camera
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['--x', '0.126', '--y', '0', '--z', '0',
                   '--yaw', '-1.5708', '--pitch', '0', '--roll', '-1.5708',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'camera_optical_frame']
    )

    # --- SLAM ---
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(my_pkg, 'config', 'my_slam_params.yaml'),
                   {'use_sim_time': False}],
        prefix=['nice -n 10']
    )

    # --- NAV2 ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'True',
            'use_composition': 'False',
            'params_file': os.path.join(my_pkg, 'config', 'my_nav2_params.yaml')
        }.items()
    )

    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        parameters=[{
            'size':      0.135,
            'max_tags':  4,
            'tile_size': 4,
            'tag_family': 'tag36h11',
            'publish_tag_transforms': False
        }],
        remappings=[
            ('image',       '/image_rect'),
            ('camera_info', '/camera_info_rect'),
            ('tag_detections', '/tag_detections')
        ],
    )

    h264_encoder_node = ComposableNode(
        name='h264_encoder',
        package='isaac_ros_h264_encoder',
        plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
        parameters=[{
            'input_width': 1640,
            'input_height': 1232,
        }],
        remappings=[
            ('image_raw', '/image_rect'),
            ('image_compressed', '/video_stream/h264')
        ]
    )

    nitros_container = ComposableNodeContainer(
        name='nitros_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            camera_node,
            rectify_node,
            apriltag_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'ERROR']
    )

    tracker_node = Node(
        package='parking',
        executable='apriltag_tracker',
        name='apriltag_tracker',
    )

    explore_node = Node(
        package='frontier_exploration',
        executable='exploration_node',
        output='screen',
        remappings=[
            ('/goal_pose', '/exploration_goal')
        ]
    )

    box_estimator_node = Node(
        package='parking',
        executable='box_estimator',
        output='screen'
    )

    mission_control_node = Node(
        package='parking',
        executable='mission_controller',
        output='screen'
    )

    scan_republisher_node = Node(
        package='parking',
        executable='scan_republisher_cpp',
        output='screen'
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan_nav',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
            'use_sim_time': False
        }],
        remappings=[('/tf', '/tf_garbage')],
        arguments=['--ros-args', '--log-level', 'FATAL']
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(my_pkg, 'config', 'ekf.yaml')],
       remappings=[('odometry/filtered', '/odom')]
    )

    goal_masking_node = Node(
        package='parking',
        executable='goal_masking_node',
        name='goal_masking_node',
        output='screen',
        remappings=[
            ('/goal_pose', '/parking_goal')
        ]
    )

    velocity_controller_node = Node(
        package='parking',
        executable='lidar_velocity_controller',
        name='lidar_velocity_controller',
        output='screen',
        parameters=[{
            'serial_port': '/dev/esp32',
            'max_linear_speed': 0.4,
            'kp_v': 0.0,
            'ki_v': 0.0,
            'kd_v': 0.0,
            'kp_w': 0.0,
            'ki_w': 0.0,
            'kd_w': 0.0,
        }]
    )

    video_streamer_node = Node(
        package='parking',
        executable='video_streamer',
        name='video_streamer',
        output='screen'
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
        }]
    )

    return LaunchDescription([
        # --- ACTIVE FOR VISION TEST ---
        camera_tf,
        nitros_container,
        TimerAction(period=3.0, actions=[tracker_node]),
        TimerAction(period=5.0, actions=[box_estimator_node]),
        TimerAction(period=8.0, actions=[OpaqueFunction(function=pin_processes)]),

        # --- INACTIVE (COMMENTED OUT) ---
        # lidar_tf,
        # lidar_ghost_tf,
        # lidar_launch,
        # scan_republisher_node,
        # goal_masking_node,
        # velocity_controller_node,
        # video_streamer_node,
        # TimerAction(period=2.0, actions=[rf2o_node]),
        # TimerAction(period=4.0, actions=[robot_localization_node]),
        # TimerAction(period=6.0, actions=[slam_node]),
        # TimerAction(period=8.0, actions=[nav2_launch]),
        # TimerAction(period=25.0, actions=[mission_control_node]),
    ])
