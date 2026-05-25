#!/usr/bin/env python3
"""
mppi_obstacle_tester.py — MPPI Local Planner Obstacle Avoidance Test Harness
=============================================================================
Master's thesis evaluation tool for the MPPI local planner.

Menu:
  1 — Static Obstacle Test  : drive to (5.0, 0.0) with a fixed obstacle
  2 — Dynamic Obstacle Test : drive to (5.0, 0.0), user walks in front
  3 — Plot Results          : generate publication-ready graphs from saved data

Active telemetry during navigation:
  /odometry/filtered  → trajectory (X, Y)
  /cmd_vel            → MPPI commanded velocity (vx, wz)
  /scan               → LiDAR (frontal detection + minimum clearance)

Key metrics computed:
  T_detected   : first moment an obstacle cluster enters the frontal arc (±30°)
                 within DETECTION_RANGE_M metres
  T_reacted    : first moment MPPI brakes (vx drops to <50% of baseline) or
                 steers sharply (|wz| > EVADE_WZ_THRESHOLD rad/s)
  Reaction time: T_reacted − T_detected  (milliseconds)
  Min clearance: absolute minimum LiDAR range across all angles, all time

Topic names — edit these if your namespace differs:
  ODOM_TOPIC   : /odometry/filtered  (try /odom if EKF remaps it)
  CMDVEL_TOPIC : /cmd_vel
  SCAN_TOPIC   : /scan
"""

import collections
import json
import math
import os
import sys
import threading
import time
from datetime import datetime

import matplotlib
matplotlib.use('Agg')
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                        QoSProfile, QoSReliabilityPolicy)

import tf2_ros
import tf_transformations as tft
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# ── Topic names ───────────────────────────────────────────────────────────────
ODOM_TOPIC   = '/odometry/filtered'  # change to '/odom' if EKF remaps it
CMDVEL_TOPIC = '/cmd_vel'
SCAN_TOPIC   = '/scan'

# ── Navigation goal ───────────────────────────────────────────────────────────
GOAL_DIST = 2.5   # metres straight ahead of the robot's current AMCL pose

# ── Obstacle detection: frontal LiDAR arc ────────────────────────────────────
# lidar_tf has yaw=+90° (base_link→base_laser), so scan 0° = robot LEFT.
# Robot FORWARD corresponds to scan angle -90°.
FRONTAL_CENTER_DEG  = -90.0
FRONTAL_HALF_DEG    = 30.0    # ±30° arc either side of forward
FRONTAL_CLUSTER_MIN = 3       # min points inside arc to confirm detection (noise filter)
DETECTION_RANGE_M   = 2.0     # obstacle threshold distance (metres)

# ── MPPI reaction detection ───────────────────────────────────────────────────
BRAKE_FRACTION      = 0.5     # reaction if vx < this × pre-detection baseline
EVADE_WZ_THRESHOLD  = 0.3     # reaction if |wz| exceeds this (rad/s)
BASELINE_SAMPLES    = 20      # rolling /cmd_vel window for baseline vx estimate
VX_MOVING_MIN       = 0.05    # vx threshold to consider robot 'in motion'

# ── Results ───────────────────────────────────────────────────────────────────
RESULTS_DIR = os.path.expanduser('~/ros2_ws/src/parking/results/mppi_tests')
DATA_FILE   = os.path.join(RESULTS_DIR, 'mppi_test_data.json')

# ── Dark-theme colours (consistent with rest of project) ─────────────────────
_BG    = '#0d1117'
_CARD  = '#161b22'
_GRID  = '#21262d'
_MUTED = '#8b949e'
_BLUE  = '#58a6ff'
_CORAL = '#f78166'
_GREEN = '#3fb950'
_RED   = '#ff4444'
_YELL  = '#e3b341'
# ─────────────────────────────────────────────────────────────────────────────


# ── MetricsNode ───────────────────────────────────────────────────────────────

class MetricsNode(Node):
    """
    Subscribes to odometry, cmd_vel, and LiDAR while a navigation goal is
    active. All mutable state is protected by _lock for thread safety —
    the scan callback runs at LiDAR frequency (10–20 Hz) on the metrics
    executor thread while the main thread polls navigation status.
    """

    def __init__(self):
        super().__init__('mppi_metrics_node')
        self._lock = threading.Lock()

        # Per-run state (reset by start_run)
        self._recording    = False
        self._t0           = 0.0
        self._run_type     = ''
        self._trajectory:  list[dict] = []   # [{t, x, y}, ...]
        self._velocities:  list[dict] = []   # [{t, vx, wz}, ...]
        self._scan_mins:   list[dict] = []   # [{t, min_dist}, ...]

        # Detection + reaction timestamps (elapsed seconds from t0)
        self._t_detected:  float | None = None
        self._t_reacted:   float | None = None
        self._min_clearance = float('inf')

        # Baseline velocity state
        self._vx_buf         = collections.deque(maxlen=BASELINE_SAMPLES)
        self._baseline_vx    = 0.0
        self._baseline_locked = False   # True once T_detected fires
        self._robot_moving   = False

        # Latest AMCL pose (map frame)
        self._amcl_x:   float | None = None
        self._amcl_y:   float | None = None
        self._amcl_yaw: float | None = None

        # ── QoS — match publisher QoS or default to BEST_EFFORT ───────────
        be_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        amcl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Odometry,  ODOM_TOPIC,   self._odom_cb,   be_qos)
        self.create_subscription(Twist,     CMDVEL_TOPIC, self._cmdvel_cb, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC,   self._scan_cb,   be_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, amcl_qos
        )

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info(
            f'MetricsNode ready | {ODOM_TOPIC} | {CMDVEL_TOPIC} | {SCAN_TOPIC}'
        )

    # ── Run lifecycle ─────────────────────────────────────────────────────────

    def start_run(self, run_type: str) -> None:
        with self._lock:
            self._run_type        = run_type
            self._t0              = time.monotonic()
            self._trajectory      = []
            self._velocities      = []
            self._scan_mins       = []
            self._t_detected      = None
            self._t_reacted       = None
            self._min_clearance   = float('inf')
            self._vx_buf.clear()
            self._baseline_vx     = 0.0
            self._baseline_locked = False
            self._robot_moving    = False
            self._recording       = True

    def stop_run(self) -> dict:
        with self._lock:
            self._recording = False
            rt_ms = None
            if self._t_detected is not None and self._t_reacted is not None:
                rt_ms = (self._t_reacted - self._t_detected) * 1000.0
            mc = self._min_clearance if self._min_clearance < float('inf') else None
            return {
                'type':          self._run_type,
                'timestamp':     datetime.now().isoformat(),
                'trajectory':    list(self._trajectory),
                'velocities':    list(self._velocities),
                'scan_mins':     list(self._scan_mins),
                't_detected':    self._t_detected,
                't_reacted':     self._t_reacted,
                'reaction_ms':   rt_ms,
                'min_clearance': mc,
            }

    # ── AMCL pose ─────────────────────────────────────────────────────────────

    def _amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        o = msg.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([o.x, o.y, o.z, o.w])
        with self._lock:
            self._amcl_x   = msg.pose.pose.position.x
            self._amcl_y   = msg.pose.pose.position.y
            self._amcl_yaw = yaw

    def get_amcl_pose(self) -> tuple[float, float, float] | None:
        with self._lock:
            if self._amcl_x is None:
                return None
            return self._amcl_x, self._amcl_y, self._amcl_yaw

    # ── TF map-frame pose ─────────────────────────────────────────────────────

    def _get_map_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            return (tf.transform.translation.x,
                    tf.transform.translation.y)
        except Exception:
            return None

    # ── /odometry/filtered callback ───────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            if not self._recording:
                return
        map_pose = self._get_map_pose()
        if map_pose is None:
            return
        x, y = map_pose
        with self._lock:
            if not self._recording:
                return
            t = time.monotonic() - self._t0
            self._trajectory.append({
                't': round(t, 4),
                'x': round(x, 4),
                'y': round(y, 4),
            })

    # ── /cmd_vel callback ─────────────────────────────────────────────────────

    def _cmdvel_cb(self, msg: Twist) -> None:
        with self._lock:
            if not self._recording:
                return

            vx = msg.linear.x
            wz = msg.angular.z
            t  = time.monotonic() - self._t0

            self._velocities.append({'t': round(t, 4), 'vx': round(vx, 4),
                                     'wz': round(wz, 4)})

            # Mark moving once forward velocity sustained
            if vx >= VX_MOVING_MIN:
                self._robot_moving = True

            # Build rolling baseline before detection fires
            if not self._baseline_locked and self._robot_moving:
                self._vx_buf.append(vx)

            # ── Reaction detection (only armed after T_detected) ───────────
            if (self._t_detected is not None
                    and self._t_reacted is None
                    and self._robot_moving
                    and self._baseline_vx > VX_MOVING_MIN):

                braking = vx < self._baseline_vx * BRAKE_FRACTION
                evading = abs(wz) > EVADE_WZ_THRESHOLD

                if braking or evading:
                    self._t_reacted = t
                    reason = 'braking' if braking else 'evasion'
                    self.get_logger().info(
                        f'[REACTION] T={t:.3f}s  vx={vx:.3f} m/s  '
                        f'wz={wz:.3f} rad/s  ({reason})'
                    )

    # ── /scan callback ────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan) -> None:
        with self._lock:
            if not self._recording:
                return

            n = len(msg.ranges)
            if n == 0:
                return

            t = time.monotonic() - self._t0

            # ── Global minimum clearance (all angles, whole run) ───────────
            valid = [r for r in msg.ranges
                     if msg.range_min < r < msg.range_max]
            if valid:
                run_min = min(valid)
                self._scan_mins.append({'t': round(t, 4), 'min': round(run_min, 4)})
                if run_min < self._min_clearance:
                    self._min_clearance = run_min

            # ── Frontal arc detection — one-shot per run ───────────────────
            if self._t_detected is not None or not self._robot_moving:
                return

            center = math.radians(FRONTAL_CENTER_DEG)
            half   = math.radians(FRONTAL_HALF_DEG)
            lo, hi = center - half, center + half

            hits = 0
            for i, r in enumerate(msg.ranges):
                if not (msg.range_min < r < msg.range_max):
                    continue
                angle = msg.angle_min + i * msg.angle_increment
                # Normalise to [-π, π] to handle wrap-around
                angle = math.atan2(math.sin(angle), math.cos(angle))
                if lo <= angle <= hi and r <= DETECTION_RANGE_M:
                    hits += 1
                    if hits >= FRONTAL_CLUSTER_MIN:
                        break

            if hits >= FRONTAL_CLUSTER_MIN:
                self._t_detected = t
                # Snapshot baseline at detection moment
                if self._vx_buf:
                    self._baseline_vx = float(np.mean(list(self._vx_buf)))
                self._baseline_locked = True
                self.get_logger().info(
                    f'[DETECTED] T={t:.3f}s  '
                    f'baseline_vx={self._baseline_vx:.3f} m/s  '
                    f'cluster_hits={hits}'
                )

    # ── Live status (polled by main thread during navigation) ─────────────────

    def get_live_status(self) -> tuple:
        with self._lock:
            return (self._t_detected, self._t_reacted, self._min_clearance)


# ── Navigation helpers ────────────────────────────────────────────────────────

def _make_pose(x: float, y: float, yaw_rad: float,
               frame: str, nav: BasicNavigator) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp    = nav.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    q = tft.quaternion_from_euler(0.0, 0.0, yaw_rad)
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    return p


def _run_test(run_type: str, metrics: MetricsNode,
              nav: BasicNavigator) -> dict | None:
    """
    Execute one navigation test run. Blocks until Nav2 reports complete or
    the user aborts with Ctrl+C. MetricsNode callbacks continue firing on
    their own executor thread throughout.
    """
    # Use current AMCL pose — do NOT reset to (0,0,0) which would put the
    # robot at an invalid map location and cause the planner to fail.
    print('\n  Reading current AMCL pose...')
    start_pose = None
    t0 = time.time()
    while start_pose is None:
        start_pose = metrics.get_amcl_pose()
        if start_pose is None:
            print(f'\r  Waiting for AMCL... {time.time()-t0:.0f}s', end='', flush=True)
            time.sleep(0.5)
    x0, y0, yaw0 = start_pose
    print(f'\n  Start: x={x0:.3f}  y={y0:.3f}  θ={math.degrees(yaw0):.1f}°')

    goal_x = x0 + GOAL_DIST * math.cos(yaw0)
    goal_y = y0 + GOAL_DIST * math.sin(yaw0)
    print(f'  Goal:  x={goal_x:.3f}  y={goal_y:.3f}  '
          f'({GOAL_DIST} m ahead, same heading)')

    goal = _make_pose(goal_x, goal_y, yaw0, 'map', nav)

    metrics.start_run(run_type)
    nav.goToPose(goal)

    print('  Navigating...  (Ctrl+C to abort)\n')
    try:
        while not nav.isTaskComplete():
            t_det, t_reac, mc = metrics.get_live_status()
            det_s  = f'{t_det:.2f}s'  if t_det  is not None else '—'
            reac_s = f'{t_reac:.2f}s' if t_reac is not None else '—'
            mc_s   = f'{mc:.3f} m'    if mc < float('inf') else '—'
            print(
                f'\r  T_detected={det_s:>8s}  '
                f'T_reacted={reac_s:>8s}  '
                f'min_clearance={mc_s}   ',
                end='', flush=True,
            )
            time.sleep(0.2)
    except KeyboardInterrupt:
        print('\n  Aborted — cancelling goal...')
        nav.cancelTask()

    print()
    data = metrics.stop_run()
    data['goal_x'] = goal_x
    data['goal_y'] = goal_y
    data['start_x'] = x0
    data['start_y'] = y0

    # ── Terminal summary ───────────────────────────────────────────────────
    result = nav.getResult()
    SEP = '─' * 52
    print(f'\n{SEP}')
    print(f'  {run_type.upper()} TEST COMPLETE')
    print(SEP)
    print(f'  Nav2 result     : {result.name if result else "UNKNOWN"}')

    t_det = data['t_detected']
    t_rea = data['t_reacted']
    rt    = data['reaction_ms']
    mc    = data['min_clearance']

    if t_det is not None:
        print(f'  T_detected      : {t_det:.3f} s')
    else:
        print('  T_detected      : no obstacle in frontal arc')

    if t_rea is not None:
        print(f'  T_reacted       : {t_rea:.3f} s')
        print(f'  Reaction time   : {rt:.1f} ms')
    else:
        print('  T_reacted       : no reaction event detected')

    print(f'  Min clearance   : {mc:.4f} m' if mc is not None
          else '  Min clearance   : n/a')
    print(f'  Path points     : {len(data["trajectory"])}')
    print(f'  Velocity samples: {len(data["velocities"])}')
    print(SEP)
    return data


# ── Data persistence ──────────────────────────────────────────────────────────

def _load_data() -> list[dict]:
    if not os.path.isfile(DATA_FILE):
        return []
    with open(DATA_FILE, 'r') as f:
        return json.load(f)


def _save_data(runs: list[dict]) -> None:
    os.makedirs(RESULTS_DIR, exist_ok=True)
    with open(DATA_FILE, 'w') as f:
        json.dump(runs, f, indent=2)
    print(f'  Data saved → {DATA_FILE}')


# ── Plotting ──────────────────────────────────────────────────────────────────

_RUN_COLOURS = {'static': _BLUE, 'dynamic': _CORAL}


def _style_ax(ax) -> None:
    ax.set_facecolor(_CARD)
    ax.grid(True, color=_GRID, linewidth=0.5, linestyle='--')
    for sp in ax.spines.values():
        sp.set_color(_GRID)
    ax.tick_params(colors=_MUTED, labelsize=9)
    ax.xaxis.label.set_color(_MUTED)
    ax.yaxis.label.set_color(_MUTED)
    ax.title.set_color('white')


def _plot_results(runs: list[dict]) -> None:
    # Use the most recent static and dynamic runs
    static_run  = next((r for r in reversed(runs) if r['type'] == 'static'),  None)
    dynamic_run = next((r for r in reversed(runs) if r['type'] == 'dynamic'), None)
    active      = [r for r in [static_run, dynamic_run] if r is not None]

    if not active:
        print('  No runs available. Complete at least one test first.')
        return

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    os.makedirs(RESULTS_DIR, exist_ok=True)

    # ════════════════════════════════════════════════════════════════════════
    # Graph 1 — 2D Evasion Trajectories
    # ════════════════════════════════════════════════════════════════════════
    fig1, ax1 = plt.subplots(figsize=(10, 7))
    fig1.patch.set_facecolor(_BG)
    _style_ax(ax1)
    ax1.set_title('MPPI Obstacle Avoidance — Evasion Trajectories',
                  fontsize=13, pad=10, color='white')
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')

    # Goal marker — use most recent run's stored goal
    ref_run = active[-1]
    gx = ref_run.get('goal_x', 0.0)
    gy = ref_run.get('goal_y', 0.0)
    ax1.plot(gx, gy, marker='*', markersize=18,
             color=_YELL, zorder=5, label=f'Goal ({gx:.2f}, {gy:.2f})')

    annotation_lines = []
    for run in active:
        traj = run['trajectory']
        if not traj:
            continue
        xs = [p['x'] for p in traj]
        ys = [p['y'] for p in traj]
        c  = _RUN_COLOURS.get(run['type'], _BLUE)

        ax1.plot(xs, ys, color=c, linewidth=2.2,
                 label=f'{run["type"].capitalize()} path', zorder=3)
        ax1.plot(xs[0],  ys[0],  'o', color=_GREEN, markersize=9, zorder=6)   # start
        ax1.plot(xs[-1], ys[-1], 's', color=c,      markersize=9, zorder=6)   # end

        mc = run.get('min_clearance')
        annotation_lines.append(
            f'{run["type"].capitalize()}: min clearance = '
            + (f'{mc:.4f} m' if mc is not None else 'n/a')
        )

    if annotation_lines:
        ax1.text(
            0.02, 0.97, '\n'.join(annotation_lines),
            transform=ax1.transAxes,
            fontsize=9, color='white', verticalalignment='top',
            bbox=dict(boxstyle='round,pad=0.5',
                      facecolor=_CARD, edgecolor=_GRID, alpha=0.92),
        )

    ax1.legend(facecolor=_CARD, edgecolor=_GRID, labelcolor='white', fontsize=9)
    ax1.set_aspect('equal', adjustable='datalim')
    fig1.tight_layout()
    p1 = os.path.join(RESULTS_DIR, f'graph1_trajectories_{ts}.png')
    fig1.savefig(p1, dpi=150, facecolor=_BG, bbox_inches='tight')
    plt.close(fig1)
    print(f'  Graph 1 → {p1}')

    # ════════════════════════════════════════════════════════════════════════
    # Graph 2 — MPPI Reflex Profile (velocity time-series)
    # ════════════════════════════════════════════════════════════════════════
    fig2, ax2 = plt.subplots(figsize=(13, 6))
    fig2.patch.set_facecolor(_BG)
    _style_ax(ax2)
    ax2.set_title('MPPI Reflex Profile — Commanded Linear Velocity vs Time',
                  fontsize=13, pad=10, color='white')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Commanded vx (m/s)')

    # xaxis_transform: data coords for x, axes (0–1) for y — stable for vlines
    xax_t = ax2.get_xaxis_transform()

    legend_patches = []
    for run in active:
        vels = run['velocities']
        if not vels:
            continue
        ts_v = [v['t']  for v in vels]
        vx_v = [v['vx'] for v in vels]
        c    = _RUN_COLOURS.get(run['type'], _BLUE)
        lbl  = run['type'].capitalize()

        ax2.plot(ts_v, vx_v, color=c, linewidth=1.8,
                 label=f'{lbl} vx', alpha=0.9, zorder=3)

        t_det  = run.get('t_detected')
        t_reac = run.get('t_reacted')
        rt_ms  = run.get('reaction_ms')

        if t_det is not None:
            ax2.axvline(t_det, color=_RED, linestyle='--',
                        linewidth=1.5, alpha=0.9, zorder=4)
            ax2.text(t_det, 0.97, f' ⬇ Detected\n  {lbl}',
                     transform=xax_t, color=_RED,
                     fontsize=7, va='top')

        if t_reac is not None:
            ax2.axvline(t_reac, color=_GREEN, linestyle='--',
                        linewidth=1.5, alpha=0.9, zorder=4)
            ax2.text(t_reac, 0.82, f' ⬇ Reacted\n  {lbl}',
                     transform=xax_t, color=_GREEN,
                     fontsize=7, va='top')

        if t_det is not None and t_reac is not None and rt_ms is not None:
            ax2.axvspan(t_det, t_reac, alpha=0.12, color=_YELL, zorder=2)
            legend_patches.append(
                mpatches.Patch(facecolor=_YELL, alpha=0.5,
                               label=f'{lbl} reaction: {rt_ms:.0f} ms')
            )

    handles, labels = ax2.get_legend_handles_labels()
    ax2.legend(handles=handles + legend_patches,
               facecolor=_CARD, edgecolor=_GRID,
               labelcolor='white', fontsize=9)

    fig2.tight_layout()
    p2 = os.path.join(RESULTS_DIR, f'graph2_reflex_profile_{ts}.png')
    fig2.savefig(p2, dpi=150, facecolor=_BG, bbox_inches='tight')
    plt.close(fig2)
    print(f'  Graph 2 → {p2}')


# ── Menu ──────────────────────────────────────────────────────────────────────

_MENU = """
┌──────────────────────────────────────────┐
│   MPPI Obstacle Avoidance Test Harness   │
├──────────────────────────────────────────┤
│  1 — Static Obstacle Test                │
│  2 — Dynamic Obstacle Test               │
│  3 — Plot Results                        │
│  q — Quit                                │
└──────────────────────────────────────────┘"""


def main():
    rclpy.init()

    # MetricsNode spins on its own executor thread — never blocks the
    # main thread while BasicNavigator is waiting for Nav2 responses.
    metrics      = MetricsNode()
    metrics_exec = SingleThreadedExecutor()
    metrics_exec.add_node(metrics)
    threading.Thread(
        target=metrics_exec.spin,
        daemon=True,
        name='metrics-spin',
    ).start()

    # BasicNavigator manages its own spinning via rclpy internals.
    nav = BasicNavigator()

    print('\n[MPPI Tester] Waiting for Nav2 to become active...')
    nav.waitUntilNav2Active()
    print('[MPPI Tester] Nav2 active. Ready.\n')

    runs = _load_data()
    n_static  = sum(1 for r in runs if r['type'] == 'static')
    n_dynamic = sum(1 for r in runs if r['type'] == 'dynamic')
    print(f'  Loaded {len(runs)} existing run(s) '
          f'({n_static} static, {n_dynamic} dynamic) from disk.')

    try:
        while True:
            print(_MENU)
            choice = input('Select: ').strip().lower()

            if choice == '1':
                print('\n[Static Test] Place a static obstacle somewhere between '
                      'the robot and the goal.')
                input('  Press Enter when ready...')
                data = _run_test('static', metrics, nav)
                if data:
                    runs.append(data)
                    _save_data(runs)

            elif choice == '2':
                print('\n[Dynamic Test] You will walk in front of the robot '
                      'after it begins moving.')
                input('  Press Enter when ready...')
                data = _run_test('dynamic', metrics, nav)
                if data:
                    runs.append(data)
                    _save_data(runs)

            elif choice == '3':
                print('\n[Plotting] Generating graphs from saved data...')
                _plot_results(runs)

            elif choice == 'q':
                break
            else:
                print('  Unknown option.')

    except KeyboardInterrupt:
        print('\n\nInterrupted.')
    finally:
        metrics_exec.shutdown()
        nav.destroy_node()
        metrics.destroy_node()
        rclpy.shutdown()
        print('Shutdown complete.')


if __name__ == '__main__':
    main()
