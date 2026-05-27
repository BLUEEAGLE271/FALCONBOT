#!/usr/bin/env python3
"""Manual data logger for thesis backup data.

Subscribes to /apriltag/marker_3 and logs x, z, yaw on each Enter keypress.
Results are written to ~/ros2_ws/src/parking/results/fabricated_run/marker3_capture.csv.
"""

import csv
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

RESULTS_DIR = os.path.expanduser('~/ros2_ws/src/parking/results/fabricated_run')
CSV_PATH    = os.path.join(RESULTS_DIR, 'marker3_capture.csv')
STALE_SECS  = 1.0


class ManualDataLogger(Node):

    def __init__(self):
        super().__init__('manual_data_logger')

        os.makedirs(RESULTS_DIR, exist_ok=True)

        self._csv_file   = open(CSV_PATH, 'w', newline='')
        self._writer     = csv.writer(self._csv_file)
        self._writer.writerow(['timestamp', 'status', 'x', 'z', 'yaw_rad'])
        self._csv_file.flush()

        self._lock            = threading.Lock()
        self._last_recv_time  = None
        self._last_x          = 0.0
        self._last_z          = 0.0
        self._last_yaw        = 0.0
        self._last_quat       = (0.0, 0.0, 0.0, 1.0)
        self._last_z_cam      = (0.0, 1.0)

        self.create_subscription(
            PoseStamped,
            '/apriltag/marker_3',
            self._marker_cb,
            10,
        )

        self.get_logger().info(
            f'Logging to {CSV_PATH}\n'
            'Press Enter to capture a data point. Ctrl-C to exit.'
        )

        t = threading.Thread(target=self._input_loop, daemon=True)
        t.start()

    def _marker_cb(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        z_cam_x = 2.0 * (qx * qz + qy * qw)
        z_cam_z = 1.0 - 2.0 * (qx * qx + qy * qy)
        yaw = math.atan2(z_cam_x, z_cam_z)
        with self._lock:
            self._last_recv_time = time.time()
            self._last_x         = p.x
            self._last_z         = p.z
            self._last_yaw       = yaw
            self._last_quat      = (qx, qy, qz, qw)
            self._last_z_cam     = (z_cam_x, z_cam_z)

    def _input_loop(self):
        while True:
            try:
                input()  # blocks until Enter
            except EOFError:
                break
            self._capture()

    def _capture(self):
        now = time.time()
        ts  = now

        with self._lock:
            age = (now - self._last_recv_time) if self._last_recv_time is not None else float('inf')
            if age < STALE_SECS:
                status = 'SUCCESS'
                x, z, yaw       = self._last_x, self._last_z, self._last_yaw
                quat             = self._last_quat
                z_cam_x, z_cam_z = self._last_z_cam
            else:
                status = 'FAILED'
                x = z = yaw = 0.0
                quat             = (0.0, 0.0, 0.0, 1.0)
                z_cam_x, z_cam_z = 0.0, 1.0

        self._writer.writerow([f'{ts:.6f}', status, f'{x:.6f}', f'{z:.6f}', f'{yaw:.6f}'])
        self._csv_file.flush()

        if status == 'SUCCESS':
            print(
                f'[LOGGED] {status}  x={x:.4f} m  z={z:.4f} m  yaw={yaw:.4f} rad'
                f'  (data age {age:.3f} s)'
            )
            print(
                f'  DEBUG quat: qx={quat[0]:.4f} qy={quat[1]:.4f} '
                f'qz={quat[2]:.4f} qw={quat[3]:.4f}'
            )
            print(
                f'  DEBUG tag-Z in cam: z_cam_x={z_cam_x:.4f}  z_cam_z={z_cam_z:.4f}'
                f'  → yaw = atan2({z_cam_x:.4f}, {z_cam_z:.4f}) = {yaw:.4f} rad'
            )
        else:
            print(
                f'[LOGGED] {status}  x=0.0  z=0.0  yaw=0.0'
                f'  (no fresh data — age={age:.1f} s)'
            )

    def destroy_node(self):
        self._csv_file.close()
        super().destroy_node()


def main():
    rclpy.init(args=sys.argv)
    node = ManualDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print(f'\nData saved to {CSV_PATH}')


if __name__ == '__main__':
    main()
