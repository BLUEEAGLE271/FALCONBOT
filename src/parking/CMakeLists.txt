cmake_minimum_required(VERSION 3.8)
project(parking)

# 1. Setup for C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) # Bumped to 17 for better support
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(nav_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
# New Deps for Masking Node & ArUco
find_package(geometry_msgs REQUIRED) 
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Crucial: Find the Python cmake helper
find_package(ament_cmake_python REQUIRED)

# =========================================================
# 2.1. Build the C++ Node (Velocity Controller)
# =========================================================
add_executable(lidar_velocity_controller src/lidar_velocity_controller.cpp)
ament_target_dependencies(lidar_velocity_controller rclcpp geometry_msgs nav_msgs sensor_msgs nlohmann_json)
install(TARGETS lidar_velocity_controller DESTINATION lib/${PROJECT_NAME})

# =========================================================
# 2. Build the C++ Node (High Performance Scan Rotator)
# =========================================================
add_executable(scan_republisher_cpp src/scan_republisher.cpp)
ament_target_dependencies(scan_republisher_cpp rclcpp sensor_msgs)

install(TARGETS
  scan_republisher_cpp
  DESTINATION lib/${PROJECT_NAME}
)



# =========================================================
# 2.5 Build the GOAL MASKING NODE
# =========================================================
add_executable(goal_masking_node src/goal_masking_node.cpp)
ament_target_dependencies(goal_masking_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  tf2_ros 
  tf2_geometry_msgs
)

install(TARGETS
  goal_masking_node
  DESTINATION lib/${PROJECT_NAME}
)

# =========================================================
# 2.6 Build the ROBUST ARUCO NODE (The new addition)
# =========================================================


add_executable(video_streamer src/streamer_node.cpp)
ament_target_dependencies(video_streamer
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  video_streamer
  DESTINATION lib/${PROJECT_NAME}
)


# Explicitly link OpenCV libraries (ament dependencies handle the others)



# =========================================================
# 3. Install Python Modules (The 'parking' folder)
# =========================================================
ament_python_install_package(${PROJECT_NAME})

# =========================================================
# 4. Install Python Executables 
# =========================================================
install(PROGRAMS
  parking/rectified_parking.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rectified_parking
)

install(PROGRAMS
  parking/box_estimator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME box_estimator
)

install(PROGRAMS
  parking/apriltag_tracker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME apriltag_tracker
)

install(PROGRAMS
  parking/mission_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_controller
)

install(PROGRAMS
  parking/pose_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_dashboard
)

install(PROGRAMS
  parking/cmd_vel_serial_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_serial_bridge
)

install(PROGRAMS
  parking/esp32_odom_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME esp32_odom_publisher
)

install(PROGRAMS
  parking/nav_metrics_harness.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nav_metrics_harness
)

install(PROGRAMS
  parking/mppi_obstacle_tester.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mppi_obstacle_tester
)

install(PROGRAMS
  parking/cmd_vel_plotter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_plotter
)

install(PROGRAMS
  parking/docking_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME docking_supervisor
)

install(PROGRAMS
  parking/reset_mission.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME reset_mission
)
# =========================================================
# 5. Install Launch and Config files
# =========================================================
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
