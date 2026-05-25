#!/usr/bin/env python3
"""
nav_metrics_harness.py — Navigation Accuracy Test Harness
==========================================================
Automated Nav2 test node for a Master's thesis. Measures cross-track error
(CTE) and final pose accuracy for a goal GOAL_DIST metres straight ahead of
wherever the robot currently is in the map.

Usage:
    ros2 run parking nav_metrics_harness

Sequence:
  1. Waits for AMCL to publish a localised pose on /amcl_pose.
     Place the robot anywhere in the map, then use RViz "2D Pose Estimate"
     (or publish to /initialpose) to seed AMCL. The harness reads whatever
     AMCL converges to — no hardcoded start coordinates.
  2. Locks the AMCL pose as the test start after user confirmation.
  3. Computes goal = start + GOAL_DIST m along the robot's forward heading.
  4. Sends goal to Nav2.
  5. Monitors CTE live while Nav2 drives the robot.
  6. On completion: prints final error metrics and saves CSV.

Output:
  ~/ros2_ws/src/parking/results/nav_metrics/run_<timestamp>.csv
"""

import csv
import math
import os
import threading
import time
from datetime import datetime

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)

import tf2_ros
import tf_transformations as tft
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ── Topics ────────────────────────────────────────────────────────────────────
ODOM_TOPIC    = '/odom'        # EKF output (robot_localization remaps odometry/filtered → /odom)
PLAN_TOPIC    = '/plan'        # Nav2 global plan (try '/global_plan' if no messages arrive)
AMCL_TOPIC    = '/amcl_pose'  # AMCL localised pose in map frame (PoseWithCovarianceStamped)
CMDVEL_TOPIC  = '/cmd_vel'    # MPPI commanded velocity output

HALF_TRACK = 0.0625  # WHEEL_SEPARATION / 2 = 0.125 m / 2 — for wheel speed conversion

# ── Test geometry ─────────────────────────────────────────────────────────────
GOAL_DIST = 2.0             # metres straight ahead of the robot's start pose

# ── Results ───────────────────────────────────────────────────────────────────
RESULTS_DIR = os.path.expanduser('~/ros2_ws/src/parking/results/nav_metrics')


# ── Cross-Track Error maths ───────────────────────────────────────────────────

def _cte_to_path(px: float, py: float, plan: list[tuple[float, float]]) -> float:
    """
    Perpendicular distance from point P=(px,py) to the nearest segment
    in the plan path. Uses clamped segment projection so the result is
    the true closest-point distance, not just nearest-waypoint.
    """
    if len(plan) < 2:
        return math.hypot(px - plan[0][0], py - plan[0][1]) if plan else 0.0

    min_dist = float('inf')
    for i in range(len(plan) - 1):
        ax, ay = plan[i]
        bx, by = plan[i + 1]
        abx, aby = bx - ax, by - ay
        apx, apy = px - ax, py - ay
        ab_sq = abx * abx + aby * aby
        if ab_sq < 1e-12:
            dist = math.hypot(apx, apy)
        else:
            t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_sq))
            dist = math.hypot(px - (ax + t * abx), py - (ay + t * aby))
        if dist < min_dist:
            min_dist = dist
    return min_dist


# ── Metrics subscriber node ───────────────────────────────────────────────────

class MetricsNode(Node):
    """
    Subscribes to /odom, /plan, and /amcl_pose.
    Thread-safe: _lock guards all mutable state.
    """

    def __init__(self):
        super().__init__('nav_metrics_node')
        self._lock = threading.Lock()

        # Driven path + final pose from /odom
        self._path_actual: list[tuple[float, float]] = []
        self._final_x:   float | None = None
        self._final_y:   float | None = None
        self._final_yaw: float | None = None

        # Global plan from /plan
        self._plan:          list[tuple[float, float]] = []
        self._plan_received: bool = False

        # CTE running statistics
        self._cte_peak:  float = 0.0
        self._cte_sum:   float = 0.0
        self._cte_count: int   = 0
        self._nav_active: bool = False

        # Latest AMCL localised pose
        self._amcl_x:   float | None = None
        self._amcl_y:   float | None = None
        self._amcl_yaw: float | None = None

        # Latest MPPI commanded velocity
        self._cmd_vx: float = 0.0
        self._cmd_wz: float = 0.0

        # /odom — robot_localization publishes BEST_EFFORT
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, odom_qos)

        # /plan — Nav2's planner_server publishes with VOLATILE durability.
        # TRANSIENT_LOCAL subscriber on a VOLATILE publisher = incompatible QoS → no messages.
        plan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Path, PLAN_TOPIC, self._plan_cb, plan_qos)

        # /amcl_pose — TRANSIENT_LOCAL (AMCL latches last pose for late joiners)
        amcl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, AMCL_TOPIC, self._amcl_cb, amcl_qos
        )

        self.create_subscription(Twist, CMDVEL_TOPIC, self._cmd_vel_cb, 10)

        # TF — needed to get robot position in map frame (not odom frame)
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info(
            f'MetricsNode ready — {ODOM_TOPIC} | {PLAN_TOPIC} | {AMCL_TOPIC} | {CMDVEL_TOPIC}'
        )

    # ── /amcl_pose callback ───────────────────────────────────────────────────

    def _amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        o = msg.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([o.x, o.y, o.z, o.w])
        with self._lock:
            self._amcl_x   = msg.pose.pose.position.x
            self._amcl_y   = msg.pose.pose.position.y
            self._amcl_yaw = yaw

    def _cmd_vel_cb(self, msg: Twist) -> None:
        with self._lock:
            self._cmd_vx = msg.linear.x
            self._cmd_wz = msg.angular.z

    def get_cmd_vel(self) -> tuple[float, float]:
        with self._lock:
            return self._cmd_vx, self._cmd_wz

    def get_amcl_pose(self) -> tuple[float, float, float] | None:
        """Returns (x, y, yaw_rad) in map frame, or None if not yet received."""
        with self._lock:
            if self._amcl_x is None:
                return None
            return self._amcl_x, self._amcl_y, self._amcl_yaw

    # ── TF map-frame pose ─────────────────────────────────────────────────────

    def _get_map_pose(self) -> tuple[float, float, float] | None:
        """Look up map→base_link and return (x, y, yaw) in map frame."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
            return x, y, yaw
        except Exception:
            return None

    # ── /plan callback ────────────────────────────────────────────────────────

    def _plan_cb(self, msg: Path) -> None:
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self._lock:
            self._plan          = points
            self._plan_received = True
        self.get_logger().info(
            f'Plan received: {len(points)} waypoints', throttle_duration_sec=5.0
        )

    # ── /odom callback ────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        # Use TF map→base_link so positions are in the same frame as /plan.
        # Raw /odom position is in the odom frame, which drifts from map frame
        # by however much AMCL's map→odom transform has shifted.
        map_pose = self._get_map_pose()
        if map_pose is None:
            return
        x, y, yaw = map_pose
        with self._lock:
            self._final_x   = x
            self._final_y   = y
            self._final_yaw = yaw
            if not self._nav_active:
                return
            self._path_actual.append((x, y))
            if self._plan_received and self._plan:
                cte = _cte_to_path(x, y, self._plan)
                self._cte_sum   += cte
                self._cte_count += 1
                if cte > self._cte_peak:
                    self._cte_peak = cte

    # ── Control ───────────────────────────────────────────────────────────────

    def start_recording(self) -> None:
        with self._lock:
            self._nav_active  = True
            self._path_actual = []
            self._cte_peak    = 0.0
            self._cte_sum     = 0.0
            self._cte_count   = 0

    def stop_recording(self) -> None:
        with self._lock:
            self._nav_active = False

    def get_metrics(self) -> dict:
        with self._lock:
            avg = self._cte_sum / self._cte_count if self._cte_count > 0 else 0.0
            return {
                'path_actual': list(self._path_actual),
                'plan_points': list(self._plan),
                'final_x':     self._final_x,
                'final_y':     self._final_y,
                'final_yaw':   self._final_yaw,
                'cte_peak':    self._cte_peak,
                'cte_avg':     avg,
                'cte_samples': self._cte_count,
            }

    def get_live_cte(self) -> tuple[float, int]:
        with self._lock:
            return self._cte_peak, self._cte_count


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_pose(x: float, y: float, yaw_rad: float, frame: str) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    q = tft.quaternion_from_euler(0.0, 0.0, yaw_rad)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


def _normalise_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _save_csv(
    metrics: dict,
    start:   tuple[float, float, float],
    goal:    tuple[float, float, float],
    result_str: str,
) -> str:
    os.makedirs(RESULTS_DIR, exist_ok=True)
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    fpath = os.path.join(RESULTS_DIR, f'run_{ts}.csv')

    sx, sy, syaw = start
    gx, gy, gyaw = goal
    fx  = metrics['final_x']   or 0.0
    fy  = metrics['final_y']   or 0.0
    fyw = metrics['final_yaw'] or 0.0

    with open(fpath, 'w', newline='') as f:
        w = csv.writer(f)

        w.writerow(['# Navigation Metrics Summary'])
        w.writerow(['nav_result',      result_str])
        w.writerow(['start_x_m',       f'{sx:.4f}'])
        w.writerow(['start_y_m',       f'{sy:.4f}'])
        w.writerow(['start_yaw_deg',   f'{math.degrees(syaw):.2f}'])
        w.writerow(['goal_x_m',        f'{gx:.4f}'])
        w.writerow(['goal_y_m',        f'{gy:.4f}'])
        w.writerow(['goal_yaw_deg',    f'{math.degrees(gyaw):.2f}'])
        w.writerow(['goal_dist_m',     GOAL_DIST])
        w.writerow(['final_x_m',       f'{fx:.4f}'])
        w.writerow(['final_y_m',       f'{fy:.4f}'])
        w.writerow(['final_yaw_deg',   f'{math.degrees(fyw):.2f}'])
        w.writerow(['delta_x_m',       f'{fx - gx:+.4f}'])
        w.writerow(['delta_y_m',       f'{fy - gy:+.4f}'])
        w.writerow(['delta_yaw_deg',   f'{math.degrees(_normalise_angle(fyw - gyaw)):+.2f}'])
        w.writerow(['cte_peak_m',      f'{metrics["cte_peak"]:.4f}'])
        w.writerow(['cte_avg_m',       f'{metrics["cte_avg"]:.4f}'])
        w.writerow(['cte_samples',     metrics['cte_samples']])
        w.writerow(['actual_path_pts', len(metrics['path_actual'])])
        w.writerow(['plan_waypoints',  len(metrics['plan_points'])])
        w.writerow([])

        w.writerow(['# Actual path driven (TF map->base_link, map frame)'])
        w.writerow(['actual_x_m', 'actual_y_m'])
        for px, py in metrics['path_actual']:
            w.writerow([f'{px:.4f}', f'{py:.4f}'])
        w.writerow([])

        w.writerow([f'# Nav2 global plan waypoints (from {PLAN_TOPIC})'])
        w.writerow(['plan_x_m', 'plan_y_m'])
        for px, py in metrics['plan_points']:
            w.writerow([f'{px:.4f}', f'{py:.4f}'])

    return fpath


# ── Main ──────────────────────────────────────────────────────────────────────

def main():

    # ── 1. Init ROS ───────────────────────────────────────────────────────────
    rclpy.init()

    metrics = MetricsNode()
    metrics_exec = SingleThreadedExecutor()
    metrics_exec.add_node(metrics)
    threading.Thread(target=metrics_exec.spin, daemon=True, name='metrics-spin').start()

    nav = BasicNavigator()

    # ── 2. Wait for AMCL to localise ─────────────────────────────────────────
    SEP = '─' * 58
    print(f'\n{SEP}')
    print('  STEP 1 — Localise the robot in the map')
    print(SEP)
    print('  1. Place the robot anywhere inside your lab map.')
    print('  2. In RViz: click "2D Pose Estimate", then click the')
    print('     robot\'s location on the map and drag to set heading.')
    print()
    print('     No RViz? Use this terminal command instead:')
    print('       ros2 topic pub --once /initialpose \\')
    print('         geometry_msgs/msg/PoseWithCovarianceStamped \\')
    print('         "{pose:{pose:{position:{x:1.0,y:0.5},'
          'orientation:{z:0.707,w:0.707}}}}"')
    print()
    print(f'  Waiting for AMCL pose on {AMCL_TOPIC}...')

    start_pose = None
    t0 = time.time()
    while start_pose is None:
        start_pose = metrics.get_amcl_pose()
        if start_pose is None:
            print(f'\r  Waiting... {time.time()-t0:.0f}s elapsed', end='', flush=True)
            time.sleep(1.0)

    x0, y0, yaw0 = start_pose
    print(f'\n\n  AMCL pose received:')
    print(f'    x = {x0:.3f} m')
    print(f'    y = {y0:.3f} m')
    print(f'    θ = {math.degrees(yaw0):.1f}°')
    print()
    print('  Watch RViz: wait until the particle cloud clusters tightly')
    print('  around the robot (AMCL converged), then press Enter.')
    input('  Press Enter to lock this start pose and begin the test... ')

    # Re-read after confirmation — AMCL may have refined the pose while waiting
    latest = metrics.get_amcl_pose()
    if latest is not None:
        x0, y0, yaw0 = latest

    print(f'\n  Start locked  →  x={x0:.3f}  y={y0:.3f}  θ={math.degrees(yaw0):.1f}°')

    # ── 3. Ask for goal orientation (relative to robot's start heading) ──────
    print()
    print('  Enter goal orientation relative to the robot\'s current heading.')
    print('    0   = arrive facing the same direction (straight test)')
    print('   90   = arrive facing 90° left of start heading')
    print('  -90   = arrive facing 90° right of start heading')
    print()
    while True:
        try:
            raw = input('  Goal orientation offset [degrees, default 0]: ').strip()
            angle_offset_deg = float(raw) if raw else 0.0
            break
        except ValueError:
            print('  Please enter a number (e.g. 0, 45, -90).')

    # ── 4. Compute goal (GOAL_DIST m straight ahead, rotated arrival heading) ─
    goal_x   = x0 + GOAL_DIST * math.cos(yaw0)
    goal_y   = y0 + GOAL_DIST * math.sin(yaw0)
    goal_yaw = yaw0 + math.radians(angle_offset_deg)

    print(f'\n  Goal computed →  x={goal_x:.3f}  y={goal_y:.3f}  '
          f'θ={math.degrees(goal_yaw):.1f}°  '
          f'({GOAL_DIST} m ahead, offset {angle_offset_deg:+.1f}°)\n')

    # ── 5. Wait for Nav2 ──────────────────────────────────────────────────────
    print('[1/4] Waiting for Nav2 to become active...')
    nav.waitUntilNav2Active()
    print('      Nav2 active.\n')

    # ── 6. Send goal ──────────────────────────────────────────────────────────
    print(f'[2/4] Sending goal...')
    goal_msg = _make_pose(goal_x, goal_y, goal_yaw, 'map')
    goal_msg.header.stamp = nav.get_clock().now().to_msg()
    metrics.start_recording()
    nav.goToPose(goal_msg)

    # ── 7. Monitor ────────────────────────────────────────────────────────────
    print('[3/4] Navigating...')
    print(f'  {"samples":>7}  {"CTE(m)":>8}  {"vx(m/s)":>8}  {"wz(r/s)":>8}'
          f'  {"L(m/s)":>8}  {"R(m/s)":>8}')
    try:
        while not nav.isTaskComplete():
            peak_cte, n   = metrics.get_live_cte()
            vx, wz        = metrics.get_cmd_vel()
            v_left  = vx - wz * HALF_TRACK   # v_L = vx - wz*(L/2)
            v_right = vx + wz * HALF_TRACK   # v_R = vx + wz*(L/2)
            print(
                f'\r  {n:>7d}  {peak_cte:>8.4f}  {vx:>+8.3f}  {wz:>+8.3f}'
                f'  {v_left:>+8.3f}  {v_right:>+8.3f}',
                end='', flush=True,
            )
            time.sleep(0.5)
    except KeyboardInterrupt:
        print('\n\nAborted — cancelling goal...')
        nav.cancelTask()

    print()
    metrics.stop_recording()

    # ── 8. Result ─────────────────────────────────────────────────────────────
    result = nav.getResult()
    result_str = {
        TaskResult.SUCCEEDED: 'SUCCEEDED',
        TaskResult.CANCELED:  'CANCELED',
        TaskResult.FAILED:    'FAILED',
    }.get(result, 'UNKNOWN')
    print(f'[4/4] Nav2 result: {result_str}')

    # ── 9. Print metrics ──────────────────────────────────────────────────────
    m   = metrics.get_metrics()
    fx  = m['final_x']   or 0.0
    fy  = m['final_y']   or 0.0
    fyw = m['final_yaw'] or 0.0

    dx  = fx  - goal_x
    dy  = fy  - goal_y
    dyw = _normalise_angle(fyw - goal_yaw)

    print(f'\n{SEP}')
    print('  NAVIGATION METRICS REPORT')
    print(SEP)
    print(f'  Result           : {result_str}')
    print(f'  Start pose       : x={x0:+.4f} m   y={y0:+.4f} m   θ={math.degrees(yaw0):.1f}°')
    print(f'  Goal  pose       : x={goal_x:+.4f} m   y={goal_y:+.4f} m   θ={math.degrees(goal_yaw):.1f}°')
    print(f'  Final pose       : x={fx:+.4f} m   y={fy:+.4f} m   θ={math.degrees(fyw):.1f}°')
    print(f'  ΔX  (fwd  error) : {dx:+.4f} m')
    print(f'  ΔY  (lat  error) : {dy:+.4f} m')
    print(f'  Δθ  (hdg  error) : {math.degrees(dyw):+.2f}°')
    print(f'  Peak CTE         : {m["cte_peak"]:.4f} m')
    print(f'  Average CTE      : {m["cte_avg"]:.4f} m  ({m["cte_samples"]} samples)')
    print(f'  Path points      : {len(m["path_actual"])}')
    print(f'  Plan received    : {"yes" if m["plan_points"] else "NO — check PLAN_TOPIC"}')
    print(SEP)

    if not m['plan_points']:
        print(f'\n  WARNING: No plan received on {PLAN_TOPIC}.')
        print(f'  CTE will be 0. Try changing PLAN_TOPIC to /global_plan')
        print(f'  or check: ros2 topic list | grep plan\n')

    # ── 10. Save CSV ──────────────────────────────────────────────────────────
    saved = _save_csv(m, (x0, y0, yaw0), (goal_x, goal_y, goal_yaw), result_str)
    print(f'\n  CSV saved → {saved}\n')

    # ── Cleanup ───────────────────────────────────────────────────────────────
    metrics_exec.shutdown()
    nav.destroy_node()
    metrics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
