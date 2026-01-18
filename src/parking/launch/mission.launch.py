import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- PATHS ---
    lidar_pkg = get_package_share_directory('ldlidar_stl_ros2')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    slam_pkg = get_package_share_directory('slam_toolbox')
    my_pkg = get_package_share_directory('parking')

    # --- 1. HARDWARE (Lidar + Camera + TF) ---
    # Launch LD19 Lidar
# A. Launch Sensors
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_pkg, 'launch', 'ld19.launch.py'))
    )
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(my_pkg, 'launch', 'camera.launch.py'))
    )

    # B. TF: Robot Center (base_link) -> LiDAR
    # Offset: x = -0.045 meters (4.5cm behind center)
    # Ensure 'base_laser' matches the frame_id in your ld19 launch file (check RViz if dots don't appear)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.045', '0', '0', '1.5708', '0', '0', 'base_link', 'base_laser']
    )

    lidar_ghost_tf= Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.045', '0', '0', '-1.5708', '0', '0', 'base_link', 'base_laser_nav']
    )

    # C. TF: Robot Center (base_link) -> Camera
    # Position: 0,0,0 (Assuming base_link IS the camera location)
    # Rotation: -90 Yaw, -90 Roll (To align Optical Z with Robot X)
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.095', '0', '0', '-1.5708', '0', '-1.5708', 'base_link', 'camera_optical_frame']
    )

    
    # --- 2. SLAM (Mapping) ---
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_pkg, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': os.path.join(my_pkg, 'config', 'my_slam_params.yaml')
       }.items()
    )

    # --- 3. NAV2 (Path Planning) ---
    # We delay Nav2 by 5 seconds to ensure SLAM is ready
    nav2_launch = TimerAction(
       period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': os.path.join(my_pkg, 'config', 'my_nav2_params.yaml') # Use your TEB config!
                }.items()
            )
        ]
    )

    # --- 4. CUSTOM NODES ---
    scan_republisher_node = Node(
        package='parking',
        executable='scan_republisher', # Ensure this is registered in setup.py!
        name='scan_republisher',
        output='screen'
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
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan_nav',   # <--- LISTEN TO THE GHOST TOPIC
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0
        }],
    )

    return LaunchDescription([
        # 1. Start Transforms and Sensors immediately
        lidar_tf,
        lidar_ghost_tf,
        camera_tf,
        lidar_launch,
        camera_launch,
        scan_republisher_node,
        # 2. Wait 3s for Lidar to spin up, then start Odom
        TimerAction(period=3.0, actions=[rf2o_node]),

        # 3. Wait 5s for Odom to stabilize, then start SLAM
        TimerAction(period=5.0, actions=[slam_launch]),

        # 4. Wait 10s for Map to build, then start Nav2
        TimerAction(period=10.0, actions=[nav2_launch]),

        # 5. Finally, start your logic
        TimerAction(period=15.0, actions=[
            box_estimator_node,
            mission_control_node,
            explore_node
        ])
    ])

