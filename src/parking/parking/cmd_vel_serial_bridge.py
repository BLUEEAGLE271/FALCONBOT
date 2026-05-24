#!/usr/bin/env python3
"""
cmd_vel_serial_bridge.py — /cmd_vel → ESP32 Serial Bridge
==========================================================
Subscribes to /cmd_vel (geometry_msgs/Twist) and forwards each command
to the ESP32 using the exact same serial protocol as motor_tester.py:

    TX: {"T":13,"X":<linear_x>,"Z":<angular_z>}\n   (CMD_ROS_CTRL)

A 5 Hz keepalive timer re-sends the last cached command so the ESP32
safety watchdog never zeros the motors mid-drive — same pattern as
_hold_velocity() in motor_tester.py (200 ms interval, re-sends until stop).

Serial parameters are identical to motor_tester.py:
    Port  : /dev/esp32
    Baud  : 115200
"""

import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


SERIAL_PORT = '/dev/esp32'
BAUD_RATE = 115200
KEEPALIVE_HZ = 5.0   # 200 ms — matches motor_tester._hold_velocity interval


class CmdVelSerialBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_serial_bridge')

        self._lock = threading.Lock()
        self._last_linear_x = 0.0
        self._last_angular_z = 0.0

        self._ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
        self.get_logger().info(f'Opened {SERIAL_PORT} @ {BAUD_RATE} baud')

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        # Keepalive: re-send last cached command every 200 ms to prevent ESP32 watchdog
        self.create_timer(1.0 / KEEPALIVE_HZ, self._keepalive_callback)

        self.get_logger().info('Ready — bridging /cmd_vel → ESP32 serial')

    def _cmd_vel_callback(self, msg: Twist) -> None:
        with self._lock:
            self._last_linear_x = msg.linear.x
            self._last_angular_z = msg.angular.z
        self._send(msg.linear.x, msg.angular.z)

    def _keepalive_callback(self) -> None:
        with self._lock:
            vx = self._last_linear_x
            wz = self._last_angular_z
        self._send(vx, wz)

    def _send(self, linear_x: float, angular_z: float) -> None:
        """Mirror of motor_tester._send_velocity — identical format and encoding."""
        cmd = f'{{"T":13,"X":{linear_x:.3f},"Z":{angular_z:.3f}}}\n'
        try:
            self._ser.write(cmd.encode('ascii'))
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write failed: {exc}')

    def destroy_node(self) -> None:
        self.get_logger().info('Shutdown — sending stop to ESP32')
        self._send(0.0, 0.0)
        self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
