cmake_minimum_required(VERSION 3.8)
project(parking)

# 1. Setup for C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# Crucial: Find the Python cmake helper
find_package(ament_cmake_python REQUIRED)

# =========================================================
# 2. Build the C++ Node (High Performance Scan Rotator)
# =========================================================
# Ensure the file is physically at src/parking/src/scan_republisher.cpp
add_executable(scan_republisher_cpp src/scan_republisher.cpp)
ament_target_dependencies(scan_republisher_cpp rclcpp sensor_msgs)

install(TARGETS
  scan_republisher_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# =========================================================
# 3. Install Python Modules (The 'parking' folder)
# =========================================================
# This installs the python library so you can 'import parking'
ament_python_install_package(${PROJECT_NAME})

# =========================================================
# 4. Install Python Executables (The missing link!)
# =========================================================
# We manually copy the scripts to lib/parking and rename them 
# so 'ros2 run' and your launch file can find them.

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