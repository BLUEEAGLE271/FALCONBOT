import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import subprocess, psutil
##Bash commands
##colcon build --symlink-install   --allow-overriding cv_bridge image_geometry --parallel-workers 2
##source install/setup.bash
##ros2 launch parking mission.launch.py

def pin_processes(context, *args, **kwargs):
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            cmd = ' '.join(proc.info['cmdline'] or [])
            if 'controller_server' in cmd:
                subprocess.run(['taskset', '-cp', '2,3,4,5', str(proc.pid)])
            elif 'planner_server' in cmd:
                subprocess.run(['taskset', '-cp', '2,3,4,5', str(proc.pid)])
            elif 'slam_toolbox' in cmd:
                subprocess.run(['taskset', '-cp', '0,1', str(proc.pid)])
            elif 'rf2o' in cmd:
                subprocess.run(['taskset', '-cp', '0,1', str(proc.pid)])
            elif 'aruco' in cmd:
                subprocess.run(['taskset', '-cp', '0,1', str(proc.pid)])
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return []


def generate_launch_description():
    # --- PATHS ---
    lidar_pkg = get_package_share_directory('ldlidar_stl_ros2')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    ##slam_pkg = get_package_share_directory('slam_toolbox')
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
# This node will publish the transform at 10Hz with a CURRENT timestamp
    # This solves the 0.1Hz bottleneck by satisfying the MessageFilter's time requirement
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

    # C. TF: Robot Center (base_link) -> Camera
    # Position: 0,0,0 (Assuming base_link IS the camera location)
    # Rotation: -90 Yaw, -90 Roll (To align Optical Z with Robot X)
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['--x', '0.126', '--y', '0', '--z', '0', 
                   '--yaw', '-1.5708', '--pitch', '0', '--roll', '-1.5708', 
                   '--frame-id', 'base_link', 
                   '--child-frame-id', 'camera_optical_frame']
    )

    
    # --- 2. SLAM (Mapping) ---
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(my_pkg, 'config', 'my_slam_params.yaml'), 
                   {'use_sim_time': False}],
        # THE NUCLEAR OPTION: Run with lower priority
        prefix=['nice -n 10'] 
    )

    # --- 3. NAV2 (Path Planning) ---
    # We delay Nav2 by 5 seconds to ensure SLAM is ready
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'True',   # <--- ADD THIS LINE EXACTLY LIKE THIS
            'use_composition': 'False',
            'params_file': os.path.join(my_pkg, 'config', 'my_nav2_params.yaml') # Use your TEB config!
        }.items()
    )

    #

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
        executable='scan_republisher_cpp', # The C++ executable you built
        output='screen'
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan_nav',
            'odom_topic': '/odom_rf2o',      # <--- RENAME: EKF listens to this
            'publish_tf': False,             # <--- CRITICAL: Disable TF, EKF handles it now
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

    aruco_node = Node(
        package='parking',           # Your package
        executable='robust_aruco',   # Your new C++ executable
        name='robust_aruco',
        output='screen',
        parameters=[{
            'marker_size_default': 0.094,    # IDs 0, 1, 2
            'marker_size_small': 0.0849,    # Your new ID 3 size
            'small_marker_id': 3,           # Identifies which one is unique
            'target_id': 1,
            'camera_frame': 'camera_optical_frame',
            'filter_alpha_default': 0.3,    # Standard smoothing
            'filter_alpha_small': 0.1
        }]
    )


    goal_masking_node = Node(
        package='parking',
        executable='goal_masking_node',
        name='goal_masking_node',
        output='screen',
        remappings=[
            # Remap the default listener to the topic coming from BoxEstimator
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
            'max_linear_speed': 1.0,
            'smc_lambda': 3.0,   # Equivalent to old Ki, closes the steady-state gap
            'smc_k': 0.6,        # The raw "Punch" voltage to break static friction
            'smc_phi': 0.5      # The smoothing layer to prevent 20Hz vibration
        }]
    )

    video_streamer_node = Node(
    package='parking',
    executable='video_streamer',
    name='video_streamer',
    output='screen'
    )

    video_stream_node = Node(
    package='web_video_server',
    executable='web_video_server',
    name='web_video_server',
    )

   

    return LaunchDescription([
        # 1. Start Transforms and Sensors immediately
        lidar_tf,
        lidar_ghost_tf,
        camera_tf,
        lidar_launch,
        camera_launch,
        aruco_node,
        scan_republisher_node,
        goal_masking_node,
        velocity_controller_node,
        #video_streamer_node,
        video_stream_node,
        

        
        # 2. Wait 3s for Lidar to spin up, then start Odom
        TimerAction(period=2.0, actions=[rf2o_node]),
        
        TimerAction(period=4.0, actions=[robot_localization_node]),

        # 3. Wait 5s for Odom to stabilize, then start SLAM
        TimerAction(period=6.0, actions=[slam_node]),

        # # # 4. Wait 10s for Map to build, then start Nav2
        TimerAction(period=8.0, actions=[nav2_launch]),
        TimerAction(period=15.0, actions=[OpaqueFunction(function=pin_processes)]),

        # # # 5. Finally, start your logic
        TimerAction(period=25.0, actions=[
             mission_control_node,
             box_estimator_node,
             #explore_node
         ])
    ])

