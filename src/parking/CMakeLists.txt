cmake_minimum_required(VERSION 3.8)
project(parking)

# 1. Setup for C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) # Bumped to 17 for better support, 14 is okay too
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# New Deps for Masking Node
find_package(geometry_msgs REQUIRED) 
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Crucial: Find the Python cmake helper
find_package(ament_cmake_python REQUIRED)

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
# 2.5 Build the GOAL MASKING NODE (The new C++ logic)
# =========================================================
# ASSUMPTION: You put the file at src/goal_masking_node.cpp
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
# 3. Install Python Modules (The 'parking' folder)
# =========================================================
ament_python_install_package(${PROJECT_NAME})

# =========================================================
# 4. Install Python Executables 
# =========================================================
# Since this is an ament_cmake package, we manually install the scripts.
# The setup.py 'entry_points' are largely ignored here, this block does the work.

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
  parking/mission_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_controller
)

# =========================================================
# 5. Install Launch and Config files
# =========================================================
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()