#!/usr/bin/env python3
"""
esp32_odom_publisher.py — Unified ESP32 Serial Bridge
======================================================
Single node that owns /dev/esp32 exclusively. Two responsibilities:

  RX — reads T,LeftTarget,LeftActual,RightTarget,RightActual telemetry,
       applies differential drive kinematics, integrates dead-reckoning,
       and publishes nav_msgs/Odometry to /wheel/odometry.

  TX — subscribes to /cmd_vel and forwards to ESP32 as
       {"T":13,"X":<vx>,"Z":<wz>}. A 5 Hz keepalive re-sends the last
       cached command so the ESP32 watchdog never zeros the motors.

This node supersedes cmd_vel_serial_bridge.py — do NOT run both.
The EKF (robot_localization) owns the odom→base_link TF; this node
does not publish that transform.

Differential drive parameters
  WHEEL_SEPARATION : 0.125 m  (axle length / track width)

EKF fusion role (odom1 in ekf.yaml)
  Only vx is fused — vyaw is excluded so the IMU gyro owns heading.
  Absolute pose fields (x, y, yaw) are not fused to avoid fighting RF2O.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf_transformations as tft
import serial


# ── Robot geometry ─────────────────────────────────────────────────────────────
WHEEL_SEPARATION = 0.125     # metres

# ── EMA low-pass filter ────────────────────────────────────────────────────────
# Smooths per-wheel velocity before kinematics to suppress serial/EMI spikes.
# alpha=1.0 → no filtering (raw);  alpha→0.0 → heavy smoothing + lag.
EMA_ALPHA = 0.3              # matches apriltag_tracker ALPHA_DEFAULT

# ── Serial ────────────────────────────────────────────────────────────────────
SERIAL_PORT  = '/dev/esp32'
BAUD_RATE    = 115200
KEEPALIVE_HZ = 5.0           # re-send last /cmd_vel to prevent ESP32 watchdog

# ── Topics ────────────────────────────────────────────────────────────────────
ODOM_TOPIC = '/wheel/odometry'

# ── Twist covariance diagonals (body frame, m/s or rad/s) ─────────────────────
# robot_localization uses these values from the message.
# vx  : moderate trust — encoder reliable on flat floor
# vy  : near-infinite — physically impossible for differential drive
# vyaw: not fused by EKF (IMU handles heading) but still set plausibly
_TC_VX   = 0.01
_TC_VY   = 1e6
_TC_VZ   = 1e6
_TC_VR   = 1e6
_TC_VP   = 1e6
_TC_VYAW = 0.05

# ── Pose covariance diagonals (metres or radians) ─────────────────────────────
# Not used by EKF (pose fields are false in odom1_config) but populate for
# completeness and RViz inspection. Dead-reckoning drifts so start at ~5 cm.
_PC_X    = 0.05
_PC_Y    = 0.05
_PC_Z    = 1e6
_PC_ROLL = 1e6
_PC_PITCH = 1e6
_PC_YAW  = 0.1
# ──────────────────────────────────────────────────────────────────────────────


def _make_covariance(d0, d1, d2, d3, d4, d5) -> list:
    """Return a 36-element row-major 6×6 diagonal covariance list."""
    c = [0.0] * 36
    c[0]  = d0; c[7]  = d1; c[14] = d2
    c[21] = d3; c[28] = d4; c[35] = d5
    return c


class Esp32OdomPublisher(Node):

    def __init__(self):
        super().__init__('esp32_odom_publisher')

        # ── Dead-reckoning state (protected by _dr_lock) ───────────────────
        self._x:      float       = 0.0
        self._y:      float       = 0.0
        self._theta:  float       = 0.0
        self._last_t: float | None = None
        # EMA filter state — None until first sample initialises them
        self._ema_left:  float | None = None
        self._ema_right: float | None = None
        self._dr_lock = threading.Lock()

        # ── Cached /cmd_vel for keepalive (protected by _cmd_lock) ─────────
        self._cmd_vx  = 0.0
        self._cmd_wz  = 0.0
        self._cmd_lock = threading.Lock()

        # ── Open serial port ───────────────────────────────────────────────
        try:
            self._ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
        except serial.SerialException as exc:
            self.get_logger().fatal(f'Cannot open {SERIAL_PORT}: {exc}')
            raise

        self.get_logger().info(f'Opened {SERIAL_PORT} @ {BAUD_RATE} baud')

        # ── Odometry publisher ─────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, ODOM_TOPIC, 10)

        # ── /cmd_vel subscriber ────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # ── Keepalive timer ────────────────────────────────────────────────
        self.create_timer(1.0 / KEEPALIVE_HZ, self._keepalive_cb)

        # ── Serial reader — daemon thread so it dies with the process ──────
        threading.Thread(
            target=self._reader_loop,
            daemon=True,
            name='esp32-reader'
        ).start()

        self.get_logger().info(
            f'Wheel odometry → {ODOM_TOPIC} | /cmd_vel → ESP32 serial'
        )

    # ── /cmd_vel → ESP32 TX ───────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist) -> None:
        with self._cmd_lock:
            self._cmd_vx = msg.linear.x
            self._cmd_wz = msg.angular.z
        self._serial_write(msg.linear.x, msg.angular.z)

    def _keepalive_cb(self) -> None:
        with self._cmd_lock:
            vx, wz = self._cmd_vx, self._cmd_wz
        self._serial_write(vx, wz)

    def _serial_write(self, vx: float, wz: float) -> None:
        cmd = f'{{"T":13,"X":{vx:.3f},"Z":{wz:.3f}}}\n'
        try:
            self._ser.write(cmd.encode('ascii'))
        except serial.SerialException as exc:
            self.get_logger().error(
                f'Serial write failed: {exc}', throttle_duration_sec=2.0
            )

    # ── ESP32 RX — serial reader loop (daemon thread) ─────────────────────────

    def _reader_loop(self) -> None:
        while True:
            try:
                raw = self._ser.readline()
            except serial.SerialException:
                time.sleep(0.01)
                continue

            line = raw.decode('ascii', errors='ignore').strip()

            # Only process motor telemetry; ignore IMU JSON and other lines
            if not line.startswith('T,'):
                continue

            # Format after stripping 'T,': LeftTarget,LeftActual,RightTarget,RightActual
            parts = line[2:].split(',')
            if len(parts) < 4:
                continue

            try:
                v_left  = float(parts[1])   # LeftActual  (m/s)
                v_right = float(parts[3])   # RightActual (m/s)
            except ValueError:
                continue

            self._integrate_and_publish(v_left, v_right)

    # ── Kinematics → integration → publish ────────────────────────────────────

    def _integrate_and_publish(self, v_left: float, v_right: float) -> None:
        now = time.monotonic()

        with self._dr_lock:
            if self._last_t is None:
                self._last_t = now
                return

            dt = now - self._last_t
            self._last_t = now

            # Reject impossible or stale dt (serial gap > 500 ms means something is wrong)
            if dt <= 0.0 or dt > 0.5:
                return

            # EMA low-pass filter — seed with first reading, then smooth
            if self._ema_left is None:
                self._ema_left  = v_left
                self._ema_right = v_right
            else:
                self._ema_left  = EMA_ALPHA * v_left  + (1.0 - EMA_ALPHA) * self._ema_left
                self._ema_right = EMA_ALPHA * v_right + (1.0 - EMA_ALPHA) * self._ema_right

            # Differential drive kinematics on filtered velocities
            vx   = (self._ema_left + self._ema_right) * 0.5
            vyaw = (self._ema_right - self._ema_left) / WHEEL_SEPARATION

            # Midpoint Euler integration — more accurate than forward Euler on arcs
            dtheta    = vyaw * dt
            mid_theta = self._theta + dtheta * 0.5
            self._x     += vx * math.cos(mid_theta) * dt
            self._y     += vx * math.sin(mid_theta) * dt
            self._theta += dtheta

            x, y, theta = self._x, self._y, self._theta

        # Build Odometry message outside the lock
        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'

        # Pose — accumulated dead-reckoning (not fused by EKF but useful for debug)
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        q = tft.quaternion_from_euler(0.0, 0.0, theta)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.pose.covariance = _make_covariance(
            _PC_X, _PC_Y, _PC_Z, _PC_ROLL, _PC_PITCH, _PC_YAW
        )

        # Twist — body-frame velocities (EKF fuses vx from here)
        msg.twist.twist.linear.x  = vx
        msg.twist.twist.linear.y  = 0.0
        msg.twist.twist.angular.z = vyaw
        msg.twist.covariance = _make_covariance(
            _TC_VX, _TC_VY, _TC_VZ, _TC_VR, _TC_VP, _TC_VYAW
        )

        self._odom_pub.publish(msg)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self.get_logger().info('Shutdown — sending stop command to ESP32')
        self._serial_write(0.0, 0.0)
        self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Esp32OdomPublisher()
    except serial.SerialException:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
