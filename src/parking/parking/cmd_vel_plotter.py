#!/usr/bin/env python3
"""
cmd_vel_plotter.py — /cmd_vel + CTE Recorder & Visualiser
==========================================================
Subscribes to /cmd_vel and /plan. Uses TF (map→base_link) for the robot's
true map-frame position so CTE is computed in the same frame as the path.

On Ctrl+C:
  1. Saves CSV to ~/ros2_ws/src/parking/results/cmd_vel/
  2. Generates a 4-subplot figure:
       - Linear velocity vx over time
       - Angular velocity wz over time
       - Individual wheel speeds over time
       - Cross-track error (perpendicular distance to /plan) over time

Usage:
    ros2 run parking cmd_vel_plotter
"""

import os
import csv
import math
import signal
import datetime
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros

HALF_TRACK = 0.0625   # WHEEL_SEPARATION / 2  (0.125 m / 2)
RESULTS_DIR = os.path.expanduser('~/ros2_ws/src/parking/results/cmd_vel')


def _cte_to_path(px, py, path_poses):
    """Minimum perpendicular distance from point (px, py) to the polyline."""
    if not path_poses:
        return float('nan')
    min_d = float('inf')
    pts = [(p.pose.position.x, p.pose.position.y) for p in path_poses]
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        dx, dy = bx - ax, by - ay
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-9:
            d = math.hypot(px - ax, py - ay)
        else:
            t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
            d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
        if d < min_d:
            min_d = d
    return min_d


class CmdVelRecorder(Node):
    def __init__(self):
        super().__init__('cmd_vel_plotter')

        be_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        rel_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._lock = threading.Lock()
        self._rows = []
        self._t0 = None
        self._last_vx  = 0.0
        self._last_wz  = 0.0
        self._last_cte = float('nan')
        self._path_poses = []

        # TF buffer — gives map→base_link (AMCL + EKF fused)
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb,  be_qos)
        self.create_subscription(Path,  '/plan',    self._plan_cb, rel_qos)
        self.create_timer(0.5, self._print_live)

        self.get_logger().info(
            'Recording /cmd_vel + CTE (map frame via TF) — press Ctrl+C to stop and save.'
        )

    # ------------------------------------------------------------------
    def _plan_cb(self, msg: Path):
        with self._lock:
            self._path_poses = msg.poses

    def _get_map_pose(self):
        """Return (x, y) of base_link in the map frame, or None on failure."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            return x, y
        except Exception:
            return None

    def _cmd_cb(self, msg: Twist):
        now = self.get_clock().now().nanoseconds * 1e-9
        pose = self._get_map_pose()

        with self._lock:
            if self._t0 is None:
                self._t0 = now
            t = now - self._t0
            vx = msg.linear.x
            wz = msg.angular.z
            v_left  = vx - wz * HALF_TRACK
            v_right = vx + wz * HALF_TRACK

            if pose is not None:
                cte = _cte_to_path(pose[0], pose[1], self._path_poses)
            else:
                cte = float('nan')

            self._rows.append((t, vx, wz, v_left, v_right, cte))
            self._last_vx  = vx
            self._last_wz  = wz
            self._last_cte = cte

    # ------------------------------------------------------------------
    def _print_live(self):
        with self._lock:
            n   = len(self._rows)
            vx  = self._last_vx
            wz  = self._last_wz
            cte = self._last_cte
        v_left  = vx - wz * HALF_TRACK
        v_right = vx + wz * HALF_TRACK
        cte_str = f'{cte:.3f} m' if not math.isnan(cte) else '  n/a  '
        print(
            f'\r  samples={n:5d}  '
            f'vx={vx:+.3f}  wz={wz:+.4f}  '
            f'v_L={v_left:+.3f}  v_R={v_right:+.3f}  '
            f'CTE={cte_str}   ',
            end='', flush=True
        )

    # ------------------------------------------------------------------
    def save_and_plot(self):
        with self._lock:
            rows = list(self._rows)

        if not rows:
            print('\nNo data recorded — nothing to save.')
            return

        os.makedirs(RESULTS_DIR, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path  = os.path.join(RESULTS_DIR, f'cmd_vel_{stamp}.csv')
        plot_path = os.path.join(RESULTS_DIR, f'cmd_vel_{stamp}.png')

        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'vx_m_s', 'wz_rad_s',
                             'v_left_m_s', 'v_right_m_s', 'cte_m'])
            writer.writerows(rows)
        print(f'\nCSV saved → {csv_path}')

        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            times    = [r[0] for r in rows]
            vxs      = [r[1] for r in rows]
            wzs      = [r[2] for r in rows]
            v_lefts  = [r[3] for r in rows]
            v_rights = [r[4] for r in rows]
            ctes     = [r[5] for r in rows]

            fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
            fig.suptitle(f'MPPI /cmd_vel + CTE (map frame)  ({stamp})', fontsize=13)

            axes[0].plot(times, vxs, color='steelblue', linewidth=1.2)
            axes[0].set_ylabel('vx  [m/s]')
            axes[0].axhline(0, color='grey', linewidth=0.5, linestyle='--')
            axes[0].grid(True, alpha=0.4)

            axes[1].plot(times, wzs, color='darkorange', linewidth=1.2)
            axes[1].set_ylabel('wz  [rad/s]')
            axes[1].axhline(0, color='grey', linewidth=0.5, linestyle='--')
            axes[1].grid(True, alpha=0.4)

            axes[2].plot(times, v_lefts,  color='forestgreen', linewidth=1.2, label='v_left')
            axes[2].plot(times, v_rights, color='crimson',      linewidth=1.2, label='v_right')
            axes[2].set_ylabel('Wheel speed  [m/s]')
            axes[2].axhline(0, color='grey', linewidth=0.5, linestyle='--')
            axes[2].legend(loc='upper right', fontsize=9)
            axes[2].grid(True, alpha=0.4)

            cte_t = [t for t, c in zip(times, ctes) if not math.isnan(c)]
            cte_v = [c for c in ctes if not math.isnan(c)]
            if cte_t:
                axes[3].plot(cte_t, cte_v, color='purple', linewidth=1.2)
                peak = max(cte_v)
                avg  = sum(cte_v) / len(cte_v)
                axes[3].axhline(peak, color='red',  linewidth=0.8, linestyle='--',
                                label=f'peak={peak:.3f} m')
                axes[3].axhline(avg,  color='grey', linewidth=0.8, linestyle=':',
                                label=f'avg={avg:.3f} m')
                axes[3].legend(loc='upper right', fontsize=9)
            axes[3].set_ylabel('CTE  [m]\n(map frame)')
            axes[3].set_xlabel('Time  [s]')
            axes[3].grid(True, alpha=0.4)

            plt.tight_layout()
            fig.savefig(plot_path, dpi=150)
            plt.close(fig)
            print(f'Plot saved  → {plot_path}')
        except ImportError:
            print('matplotlib not available — skipping plot.')


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRecorder()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    def _shutdown(sig, frame):
        print('\nStopping...')
        node.save_and_plot()
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    spin_thread.join()


if __name__ == '__main__':
    main()
