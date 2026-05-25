#!/usr/bin/env python3
"""
play_mapping.py — Phase 0: Manual Teleoperation for SLAM Map Generation
========================================================================
This script is a thin automation wrapper around the standard ROS 2 teleop
node. It does NOT re-implement motor control; instead it delegates entirely
to `teleop_twist_keyboard`, which publishes geometry_msgs/Twist to /cmd_vel.
The C++ `lidar_velocity_controller` node (already in mission.launch.py)
subscribes to /cmd_vel and forwards the command to the ESP32 over serial
as: {"T":13,"X":<linear_x>,"Z":<angular_z>}

Pipeline position:
  This script sits at the very start of the test pipeline:
    Phase 0  →  play_mapping.py  (you are here)
    Phase 1  →  autonomous navigation tests using the saved map
    Phase N  →  motor_tester.py / velocity_tester.py for characterisation

Prerequisites (all must be running in separate terminals before this script):
  1. ros2 launch ldlidar_stl_ros2 ld19.launch.py
  2. ros2 run tf2_ros static_transform_publisher ...   (lidar TFs)
  3. ros2 run rf2o_laser_odometry rf2o_laser_odometry_node ...
  4. ros2 run robot_localization ekf_node ...
  5. ros2 run slam_toolbox sync_slam_toolbox_node --ros-args -p use_sim_time:=false ...
  6. ros2 run parking lidar_velocity_controller    (serial bridge /cmd_vel → ESP32)

Usage:
    python3 play_mapping.py
"""

import subprocess
import sys

_BANNER = r"""
╔══════════════════════════════════════════════════════════════════════╗
║                                                                      ║
║        ██████╗ ██╗  ██╗ █████╗ ███████╗███████╗     ██████╗         ║
║        ██╔══██╗██║  ██║██╔══██╗██╔════╝██╔════╝    ██╔═████╗        ║
║        ██████╔╝███████║███████║███████╗█████╗      ██║██╔██║        ║
║        ██╔═══╝ ██╔══██║██╔══██║╚════██║██╔══╝      ████╔╝██║        ║
║        ██║     ██║  ██║██║  ██║███████║███████╗    ╚██████╔╝        ║
║        ╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝╚══════╝     ╚═════╝         ║
║                                                                      ║
║          INITIALIZING PHASE 0: AMR MAPPING MODE                      ║
║          Drive the robot around the lab to build the map.            ║
║                                                                      ║
║  Controls (standard teleop_twist_keyboard):                          ║
║    u  i  o      Forward / Forward+Turn                               ║
║    j  k  l      Rotate Left / Stop / Rotate Right                    ║
║    m  ,  .      Backward / Backward+Turn                             ║
║    q / z        Increase / decrease max speeds by 10%                ║
║    w / x        Increase / decrease linear speed only                ║
║    e / c        Increase / decrease angular speed only               ║
║                                                                      ║
║  IMPORTANT: Keep the ESP32 velocity controller running!              ║
║    The teleop node publishes /cmd_vel (Twist). The                   ║
║    lidar_velocity_controller node bridges that to the ESP32          ║
║    serial command: {"T":13,"X":<vx>,"Z":<wz>}                        ║
║                                                                      ║
║  When finished: press Ctrl+C here to stop this script,               ║
║  then save the map with the command shown below.                     ║
╚══════════════════════════════════════════════════════════════════════╝

  Map save command (run in a NEW terminal when done driving):
    ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/my_lab_map

"""

_TELEOP_CMD = [
    "ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard",
    "--ros-args", "-p", "turn:=5.0", "-p", "speed:=0.3"
]


def main() -> None:
    print(_BANNER)

    print("[play_mapping] Launching teleop_twist_keyboard …")
    print("[play_mapping] This terminal must stay focused to receive keystrokes.\n")

    proc = subprocess.Popen(_TELEOP_CMD)

    try:
        proc.wait()
    except KeyboardInterrupt:
        # User pressed Ctrl+C — send SIGTERM to the teleop subprocess so it
        # exits cleanly rather than leaving a dangling process.
        print("\n[play_mapping] Ctrl+C received — stopping teleop node …")
        proc.terminate()
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()

    print("[play_mapping] Teleop stopped.")
    print()
    print("  Next step — save the map:")
    print("    ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/my_lab_map")
    print()
    print("  This creates:")
    print("    ~/ros2_ws/maps/my_lab_map.yaml   — map metadata")
    print("    ~/ros2_ws/maps/my_lab_map.pgm    — occupancy grid image")
    print()


if __name__ == "__main__":
    main()
