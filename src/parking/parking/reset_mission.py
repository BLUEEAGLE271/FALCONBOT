#!/usr/bin/env python3
"""
reset_mission.py — Trigger DockingSupervisor reset from the command line.

Usage:
    ros2 run parking reset_mission
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = Node('reset_mission_client')

    client = node.create_client(Trigger, '/docking_supervisor/reset_mission')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(
            'Service /docking_supervisor/reset_mission not available — '
            'is docking_supervisor running?'
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    result = future.result()
    if result is not None:
        node.get_logger().info(
            f'Reset: success={result.success}  msg="{result.message}"'
        )
    else:
        node.get_logger().error('Service call timed out or failed')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
