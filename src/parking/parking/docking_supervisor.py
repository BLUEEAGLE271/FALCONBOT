#!/usr/bin/env python3
"""
docking_supervisor.py — AprilTag Docking State Machine
=======================================================
Subscribes to /box_center_pose (from box_estimator) and drives the robot
through a two-phase dock sequence:

  IDLE                → waiting for operator to send RViz zone goal
  COMMUTING           → Nav2 driving to RViz zone goal (tag detections ignored)
  LOCAL_SEARCH        → Archimedean spiral sweep (NavigateThroughPoses)
  BUFFERING_ROUGH     → collecting BUF_ROUGH samples to compute staging goal
  NAV_TO_STAGING      → Nav2 driving to 0.5 m standoff pose
  BUFFERING_PRECISION → marker-3 seen close (<0.6 m); collecting precision samples
  NAV_TO_DOCK         → Nav2 driving to exact box centre with shrunken footprint
  LOCKED              → docked, final combined error logged
  ABORTED             → overshoot kill-switch or spiral failure; awaiting reset

Thesis extras (non-breaking):
  • CSV logger  → ~/ros2_ws/src/parking/results/thesis_docking_metrics.csv
  • Overshoot kill-switch (NAV_TO_DOCK only): armed at 10 cm, aborts on 2 cm retreat
  • Final combined error logged to CSV on successful dock
  • RViz /goal_pose proxy: robot drives to clicked zone, then spirals only if no tag seen on arrival

Reset via: ros2 run parking reset_mission
       or: ros2 service call /docking_supervisor/reset_mission std_srvs/srv/Trigger
"""

import csv
import math
import os
import threading
import time
from datetime import datetime
from enum import Enum, auto

import psutil
import rclpy
import tf_transformations as tft
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose  # ← spiral client
from nav2_msgs.msg import SpeedLimit
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Empty, Trigger


class DockState(Enum):
    IDLE                = auto()   # waiting for operator RViz zone goal
    COMMUTING           = auto()   # driving to RViz zone; tag detections locked out
    VISUAL_HOMING       = auto()   # driving toward detected tag to get a close-range re-scan
    LOCAL_SEARCH        = auto()   # expanding spiral sweep
    BUFFERING_ROUGH     = auto()
    NAV_TO_STAGING      = auto()
    BUFFERING_PRECISION = auto()
    NAV_TO_DOCK         = auto()
    LOCKED              = auto()
    ABORTED             = auto()   # overshoot kill-switch or spiral exhausted


# ── pure helpers ──────────────────────────────────────────────────────────────

def _circular_mean(angles: list[float]) -> float:
    return math.atan2(
        sum(math.sin(a) for a in angles),
        sum(math.cos(a) for a in angles),
    )


def _outlier_mean(vals: list[float]) -> float:
    """Strip 2 highest and 2 lowest; mean of the rest."""
    if len(vals) < 6:
        return sum(vals) / len(vals)
    s = sorted(vals)
    trimmed = s[2:-2]
    return sum(trimmed) / len(trimmed)


def _pose_stamped(x: float, y: float, yaw: float, frame: str = 'odom') -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = x
    p.pose.position.y = y
    q = tft.quaternion_from_euler(0.0, 0.0, yaw)
    p.pose.orientation.x, p.pose.orientation.y = q[0], q[1]
    p.pose.orientation.z, p.pose.orientation.w = q[2], q[3]
    return p


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _gen_spiral_waypoints(node: 'DockingSupervisor',
                           x0: float, y0: float) -> list[PoseStamped]:
    """
    Archimedean spiral centred at (x0, y0).

    r(θ) = R_MAX · θ / (2π · REVS)  →  r = 0 at θ=0, r = R_MAX at θ = 2π·REVS

    Yaw is set to the tangent of the spiral so the robot always faces its
    direction of travel.  Step size π/4 gives 16 waypoints for 2 revolutions.
    """
    R_MAX = 1.5
    REVS  = 2.0
    STEP  = math.pi / 4
    total = 2.0 * math.pi * REVS          # 4π
    a     = R_MAX / total                  # r = a·θ

    now = node.get_clock().now().to_msg()
    waypoints: list[PoseStamped] = []

    theta = STEP                           # skip θ=0 (r=0, robot already there)
    while theta <= total + 1e-9:
        r  = a * theta
        x  = x0 + r * math.cos(theta)
        y  = y0 + r * math.sin(theta)

        # Tangent direction: d/dθ (r cosθ, r sinθ) = (a cosθ − r sinθ, a sinθ + r cosθ)
        tx  = a * math.cos(theta) - r * math.sin(theta)
        ty  = a * math.sin(theta) + r * math.cos(theta)
        yaw = math.atan2(ty, tx)

        p = _pose_stamped(x, y, yaw, 'odom')
        p.header.stamp = now
        waypoints.append(p)
        theta += STEP

    return waypoints   # 16 waypoints


# ── node ──────────────────────────────────────────────────────────────────────

class DockingSupervisor(Node):

    # Tuneable constants
    APPROACH_DIST_ROUGH  = 0.50   # m — staging standoff from box centre
    BUF_ROUGH            = 10     # samples before staging nav
    BUF_PRECISION        = 10     # samples before final dock nav
    PROXIMITY_TRIGGER_M          = 0.85   # m — marker-3 camera distance → BUFFERING_PRECISION
    VISUAL_HOMING_CLOSE_M        = 0.60   # m — camera distance threshold to exit visual homing
    VISUAL_HOMING_NAV_STANDOFF_M = 0.30   # m — nav goal standoff from marker in odom frame
    DOCK_SPEED_LIMIT     = 0.20   # m/s — speed cap during final approach
    OVERSHOOT_ARM_M      = 0.04   # m — arm kill-switch when robot enters this radius
    OVERSHOOT_DELTA_M    = 0.02   # m — abort if robot moves away by this much once armed

    INFLATION_RADIUS_PRECISION = 0.11    # > 0.05 inscribed (10×10 cm footprint) — no C++ crash
    INFLATION_SCALE_PRECISION  = 100.0
    INFLATION_RADIUS_NORMAL    = 0.45    # m — restored after reset
    INFLATION_SCALE_NORMAL     = 10.0    # restored after reset

    _FP_DOCK   = '[[-0.05,-0.05],[-0.05,0.05],[0.05,0.05],[0.05,-0.05]]'   # 10×10 cm
    _FP_NORMAL = '[[-0.09,-0.08],[0.09,-0.08],[0.09,0.08],[-0.09,0.08]]'   # 18×16 cm

    _CSV_DIR = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'src', 'parking', 'results',
    )

    def __init__(self):
        super().__init__('docking_supervisor')

        # ── state machine ────────────────────────────────────────────────────
        self._state = DockState.IDLE
        self._lock  = threading.Lock()

        self._rough_buf: list[tuple[float, float, float]] = []
        self._prec_buf:  list[tuple[float, float, float]] = []
        self._current_goal_handle = None

        # ── thesis data logger ───────────────────────────────────────────────
        self._csv_lock = threading.Lock()
        self._run_id   = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._CSV_PATH       = os.path.join(self._CSV_DIR, f'docking_events_{self._run_id}.csv')
        self._telem_CSV_PATH = os.path.join(self._CSV_DIR, f'docking_telemetry_{self._run_id}.csv')
        self._summary_CSV_PATH = os.path.join(self._CSV_DIR, 'docking_summary.csv')
        self._init_csv()
        self._init_telemetry_csv()

        # ── mission-end tracking ─────────────────────────────────────────────
        self._mission_logged = False
        self._run_start_time = time.time()

        # ── cmd_vel monitor ──────────────────────────────────────────────────
        self.latest_vx: float = 0.0
        self.latest_wz: float = 0.0

        # ── overshoot kill-switch state ──────────────────────────────────────
        self._reference_dock_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_odom_pose:      tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.min_distance_to_goal: float = float('inf')  # armed watermark

        # ── per-marker debug telemetry ───────────────────────────────────────
        # {marker_id: (last_seen_ns, dist_m)}
        self._marker_seen: dict[int, tuple[int, float]] = {}

        # ── callback groups ──────────────────────────────────────────────────
        cb_sub    = ReentrantCallbackGroup()
        cb_nav    = MutuallyExclusiveCallbackGroup()
        cb_spiral = MutuallyExclusiveCallbackGroup()   # separate from cb_nav
        cb_srv    = MutuallyExclusiveCallbackGroup()

        be_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── subscriptions ────────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, '/box_center_pose',
            self._on_box_pose, 10, callback_group=cb_sub,
        )
        for _mid in range(4):
            self.create_subscription(
                PoseStamped, f'/apriltag/marker_{_mid}',
                lambda msg, mid=_mid: self._on_any_marker(msg, mid),
                be_qos, callback_group=cb_sub,
            )
        self.create_subscription(
            Odometry, '/odom',
            self._on_odom, 10, callback_group=cb_sub,
        )
        self.create_subscription(
            PoseStamped, '/goal_pose',
            self._on_goal_pose, 10, callback_group=cb_sub,
        )
        self.create_subscription(
            Twist, '/cmd_vel',
            self._on_cmd_vel, 10, callback_group=cb_sub,
        )

        # ── publishers ───────────────────────────────────────────────────────
        self._cmd_pub   = self.create_publisher(Twist,      '/cmd_vel',     10)
        self._speed_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)

        # ── action clients ───────────────────────────────────────────────────
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=cb_nav,
        )
        self._spiral_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses',
            callback_group=cb_spiral,
        )

        # ── footprint + costmap-clear service clients ─────────────────────────
        self._fp_client = self.create_client(
            SetParameters, '/local_costmap/local_costmap/set_parameters',
            callback_group=cb_srv,
        )
        self._fp_global_client = self.create_client(
            SetParameters, '/global_costmap/global_costmap/set_parameters',
            callback_group=cb_srv,
        )
        self._clear_local_client = self.create_client(
            Empty, '/local_costmap/local_costmap/clear_entirely_local_costmap',
            callback_group=cb_srv,
        )
        self._clear_global_client = self.create_client(
            Empty, '/global_costmap/global_costmap/clear_entirely_global_costmap',
            callback_group=cb_srv,
        )
        self._velocity_smoother_client = self.create_client(
            SetParameters, '/velocity_smoother/set_parameters',
            callback_group=cb_srv,
        )

        # ── 1 Hz debug telemetry timer ───────────────────────────────────────
        self.create_timer(1.0, self._debug_marker3_cb, callback_group=cb_sub)

        # ── 2 Hz CSV telemetry timer (CPU/GPU/cmd_vel/state) ─────────────────
        self.create_timer(0.5, self._telemetry_cb, callback_group=cb_sub)

        # ── reset service ────────────────────────────────────────────────────
        self.create_service(
            Trigger, '~/reset_mission',
            self._on_reset, callback_group=cb_srv,
        )

        self.log_event('NODE_START', 'DockingSupervisor initialised')
        self.get_logger().info('DockingSupervisor ready — IDLE  (send RViz 2D Nav Goal to begin)')

    # ── 1 Hz debug telemetry ─────────────────────────────────────────────────

    def _debug_marker3_cb(self):
        now_ns = self.get_clock().now().nanoseconds
        visible = [
            f'Marker {mid} ({dist:.2f} m)'
            for mid, (t_ns, dist) in self._marker_seen.items()
            if (now_ns - t_ns) / 1e9 <= 1.0
        ]
        if visible:
            self.get_logger().info('[DEBUG] VISIBLE: ' + '  |  '.join(visible))

    # ── telemetry CSV init ────────────────────────────────────────────────────

    def _init_telemetry_csv(self):
        os.makedirs(self._CSV_DIR, exist_ok=True)
        with self._csv_lock:
            with open(self._telem_CSV_PATH, 'w', newline='') as f:
                csv.writer(f).writerow(
                    ['elapsed_s', 'timestamp_iso', 'state',
                     'cpu_pct', 'ram_pct', 'gpu_pct', 'cmd_vel_x', 'cmd_vel_z']
                )

    # ── GPU reader ────────────────────────────────────────────────────────────

    @staticmethod
    def _read_gpu_load() -> float:
        try:
            with open('/sys/devices/gpu.0/load') as f:
                return float(f.read().strip()) / 10.0
        except OSError:
            return 0.0

    # ── cmd_vel monitor ───────────────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist):
        self.latest_vx = msg.linear.x
        self.latest_wz = msg.angular.z

    # ── 2 Hz telemetry callback ───────────────────────────────────────────────

    def _telemetry_cb(self):
        elapsed = time.time() - self._run_start_time
        now_iso = datetime.now().strftime('%Y-%m-%dT%H:%M:%S.%f')
        with self._lock:
            state_name = self._state.name
        cpu = psutil.cpu_percent(interval=None)
        ram = psutil.virtual_memory().percent
        gpu = self._read_gpu_load()
        with self._csv_lock:
            with open(self._telem_CSV_PATH, 'a', newline='') as f:
                csv.writer(f).writerow(
                    [f'{elapsed:.2f}', now_iso, state_name,
                     f'{cpu:.1f}', f'{ram:.1f}', f'{gpu:.1f}',
                     f'{self.latest_vx:.4f}', f'{self.latest_wz:.4f}']
                )

    # ── end-of-mission summary ────────────────────────────────────────────────

    def _log_mission_end(self, result: str):
        if self._mission_logged:
            return
        self._mission_logged = True

        rx, ry, ryaw = self._last_odom_pose
        gx, gy, gyaw = self._reference_dock_pose
        got_dock_goal = (gx, gy) != (0.0, 0.0)

        if got_dock_goal:
            pose_err = math.sqrt((rx - gx)**2 + (ry - gy)**2)
            yaw_err  = abs(_wrap_pi(ryaw - gyaw))
        else:
            pose_err = yaw_err = float('nan')

        # Visible terminal banner
        sep = '=' * 56
        self.get_logger().info(
            f'\n{sep}\n'
            f'  MISSION END\n'
            f'  Result     : {result}\n'
            + (f'  Pose Error : {pose_err:.4f} m\n'
               f'  Yaw Error  : {yaw_err:.4f} rad\n'
               if got_dock_goal else
               '  (No dock goal reached — no pose error computed)\n')
            + sep
        )

        # Append to shared summary CSV (one row per run)
        write_header = (
            not os.path.exists(self._summary_CSV_PATH)
            or os.path.getsize(self._summary_CSV_PATH) == 0
        )
        with self._csv_lock:
            with open(self._summary_CSV_PATH, 'a', newline='') as f:
                w = csv.writer(f)
                if write_header:
                    w.writerow(['run_id', 'timestamp_iso', 'result',
                                'pose_error_m', 'yaw_error_rad',
                                'ref_dock_x', 'ref_dock_y', 'ref_dock_yaw_deg'])
                w.writerow([
                    self._run_id,
                    datetime.now().strftime('%Y-%m-%dT%H:%M:%S'),
                    result,
                    f'{pose_err:.6f}' if got_dock_goal else '',
                    f'{yaw_err:.6f}' if got_dock_goal else '',
                    f'{gx:.4f}', f'{gy:.4f}',
                    f'{math.degrees(gyaw):.2f}',
                ])

        # Reset velocity so next telemetry rows are clean
        self.latest_vx = 0.0
        self.latest_wz = 0.0

    # ── CSV logger ────────────────────────────────────────────────────────────

    def _init_csv(self):
        os.makedirs(self._CSV_DIR, exist_ok=True)
        with self._csv_lock:
            with open(self._CSV_PATH, 'w', newline='') as f:
                csv.writer(f).writerow(
                    ['timestamp_iso', 'event_type', 'description', 'value']
                )

    def log_event(self, event_type: str, description: str, value: str = ''):
        now = datetime.now().strftime('%Y-%m-%dT%H:%M:%S.%f')
        with self._csv_lock:
            with open(self._CSV_PATH, 'a', newline='') as f:
                csv.writer(f).writerow([now, event_type, description, value])

    # ── spiral generation + execution ─────────────────────────────────────────

    def _fire_local_search(self):
        x0, y0, _ = self._last_odom_pose
        waypoints  = _gen_spiral_waypoints(self, x0, y0)

        self.get_logger().info(
            f'Spiral: origin=({x0:.2f},{y0:.2f})  '
            f'{len(waypoints)} waypoints  r_max=1.5 m'
        )
        self.log_event(
            'STATE_CHANGE', 'Transitioned to LOCAL_SEARCH',
            f'origin=({x0:.2f},{y0:.2f}) waypoints={len(waypoints)}',
        )

        if not self._spiral_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'NavigateThroughPoses server unavailable — ABORTED'
            )
            self.log_event('SAFETY_ABORT', 'NavigateThroughPoses server unavailable')
            with self._lock:
                self._state = DockState.ABORTED
            self._stop_robot()
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = waypoints

        future = self._spiral_client.send_goal_async(goal)
        future.add_done_callback(self._spiral_accepted_cb)

    def _spiral_accepted_cb(self, future):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Spiral goal rejected — back to IDLE')
            self.log_event('ERROR', 'NavigateThroughPoses goal rejected')
            with self._lock:
                self._state = DockState.IDLE
            return

        self.log_event('GOAL_ACCEPTED', 'Spiral NavigateThroughPoses goal accepted')
        with self._lock:
            self._current_goal_handle = handle
        handle.get_result_async().add_done_callback(self._local_search_done_cb)

    def _local_search_done_cb(self, future):
        result = future.result()
        status = result.status if result else GoalStatus.STATUS_ABORTED

        with self._lock:
            state = self._state

        # Tag was detected mid-spiral: state already changed to BUFFERING_* or
        # VISUAL_HOMING by _on_box_pose / _on_marker3 — nothing left to do here.
        if state in (DockState.BUFFERING_ROUGH, DockState.BUFFERING_PRECISION,
                     DockState.VISUAL_HOMING):
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            # Robot drove the entire spiral and never detected the tag
            self.get_logger().error('Spiral complete — target not found. ABORTED.')
            self.log_event(
                'SAFETY_ABORT',
                'Target not found after expanding radial sweep',
            )
            with self._lock:
                self._state = DockState.ABORTED
            self._stop_robot()
        else:
            # Cancelled for an unexpected reason — return to IDLE for operator retry
            self.get_logger().warn(
                f'Spiral ended unexpectedly (status={status}) — IDLE'
            )
            self.log_event(
                'STATE_CHANGE', 'Transitioned to IDLE',
                f'spiral_status={status}',
            )
            with self._lock:
                self._state = DockState.IDLE

    def _zone_nav_done_cb(self, future):
        """Fires when Nav2 finishes driving to the RViz-clicked zone goal.

        SUCCEEDED + still COMMUTING → robot arrived at zone, no tag seen → start spiral.
        Any other state             → tag detected mid-drive; docking path already active.
        Cancelled / failed          → new /goal_pose arrived or Nav2 error; do nothing.
        """
        result = future.result()
        status = result.status if result else GoalStatus.STATUS_ABORTED

        with self._lock:
            state = self._state

        if state != DockState.COMMUTING:
            # Tag was detected while driving to the zone — nominal docking path is
            # already running; nothing to do here.
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                'Zone goal reached — no tag seen, starting LOCAL_SEARCH spiral'
            )
            self.log_event(
                'STATE_CHANGE', 'Transitioned to LOCAL_SEARCH',
                'zone goal reached with no tag detection',
            )
            with self._lock:
                self._state = DockState.LOCAL_SEARCH
            self._fire_local_search()
        else:
            # Preempted by a new /goal_pose or Nav2 failure — state stays COMMUTING
            self.get_logger().info(
                f'Zone nav ended (status={status}) in COMMUTING — awaiting next goal'
            )

    # ── box_center_pose callback ───────────────────────────────────────────────

    def _on_box_pose(self, msg: PoseStamped):
        """Fills BUFFERING_ROUGH and BUFFERING_PRECISION sample buffers only.

        All state transitions into/out of COMMUTING, LOCAL_SEARCH, and
        VISUAL_HOMING are driven by _on_any_marker (camera-frame distances).
        box_center_pose is only used once the robot is already close and
        collecting accurate odom-frame samples.
        """
        q = msg.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        sample = (msg.pose.position.x, msg.pose.position.y, yaw)

        fire_staging = False
        fire_dock    = False

        with self._lock:
            if self._state in (DockState.IDLE, DockState.LOCKED, DockState.ABORTED,
                               DockState.COMMUTING, DockState.LOCAL_SEARCH,
                               DockState.VISUAL_HOMING):
                return

            if self._state == DockState.BUFFERING_ROUGH:
                self._rough_buf.append(sample)
                if len(self._rough_buf) >= self.BUF_ROUGH:
                    self._state  = DockState.NAV_TO_STAGING
                    fire_staging = True

            elif self._state == DockState.BUFFERING_PRECISION:
                self._prec_buf.append(sample)
                if len(self._prec_buf) >= self.BUF_PRECISION:
                    self._state = DockState.NAV_TO_DOCK
                    fire_dock   = True

        if fire_staging:
            with self._lock:
                buf = list(self._rough_buf)
            self._fire_staging_goal(buf)

        if fire_dock:
            with self._lock:
                buf = list(self._prec_buf)
            self._fire_dock_goal(buf)

    # ── marker callbacks — visual homing + precision trigger ─────────────────

    def _marker_cam_to_odom(self, p) -> tuple[float, float]:
        """Transform a camera_optical position to odom (x, y).

        Uses the hardcoded static camera_optical→base_link transform from the
        launch file (x=0.126 m, yaw=-π/2, roll=-π/2).
        Rotation matrix:  [[0,0,1], [-1,0,0], [0,-1,0]]
          bx = p.z + 0.126   (camera Z  → robot forward + lens offset)
          by = -p.x           (camera X  → -robot lateral  [X_cam right = Y_base left])
        Then applies the robot's current yaw to get odom coordinates.
        """
        bx = p.z + 0.126
        by = -p.x
        rx, ry, ryaw = self._last_odom_pose
        ox = rx + bx * math.cos(ryaw) - by * math.sin(ryaw)
        oy = ry + bx * math.sin(ryaw) + by * math.cos(ryaw)
        return ox, oy

    def _on_any_marker(self, msg: PoseStamped, marker_id: int):
        """State-machine transitions driven entirely by camera-frame marker distance.

        Priority order (under lock):
          1. Marker-3 precision trigger  (any homing state, dist < PROXIMITY_TRIGGER_M)
          2. Visual homing entry         (COMMUTING / LOCAL_SEARCH, dist > VISUAL_HOMING_CLOSE_M)
          3. Direct BUFFERING_ROUGH      (COMMUTING / LOCAL_SEARCH, dist ≤ VISUAL_HOMING_CLOSE_M)
          4. Visual homing exit          (VISUAL_HOMING, dist ≤ VISUAL_HOMING_CLOSE_M)
        """
        p = msg.pose.position
        dist = math.sqrt(p.x**2 + p.y**2 + p.z**2)

        self._marker_seen[marker_id] = (self.get_clock().now().nanoseconds, dist)

        action:   str   = ''
        homing_ox: float = 0.0
        homing_oy: float = 0.0

        with self._lock:
            # ── 1. Marker-3 precision trigger ──────────────────────────────
            if (marker_id == 3
                    and self._state in (DockState.COMMUTING, DockState.LOCAL_SEARCH,
                                        DockState.VISUAL_HOMING)
                    and dist < self.PROXIMITY_TRIGGER_M):
                self._state = DockState.BUFFERING_PRECISION
                self._prec_buf.clear()
                action = 'precision'

            # ── 2 & 3. Visual homing entry / direct buffering ──────────────
            elif self._state in (DockState.COMMUTING, DockState.LOCAL_SEARCH):
                if dist > self.VISUAL_HOMING_CLOSE_M:
                    self._state = DockState.VISUAL_HOMING
                    action = 'homing_enter'
                    homing_ox, homing_oy = self._marker_cam_to_odom(p)
                else:
                    self._rough_buf.clear()
                    self._state = DockState.BUFFERING_ROUGH
                    action = 'buffering_direct'

            # ── 4. Visual homing exit ───────────────────────────────────────
            elif self._state == DockState.VISUAL_HOMING:
                if dist <= self.VISUAL_HOMING_CLOSE_M:
                    self._rough_buf.clear()
                    self._state = DockState.BUFFERING_ROUGH
                    action = 'homing_exit'

        # ── execute outside lock ──────────────────────────────────────────
        if action == 'precision':
            self.get_logger().info(
                f'Marker-3 at {dist:.2f} m — BUFFERING_PRECISION'
            )
            self.log_event('STATE_CHANGE', 'Transitioned to BUFFERING_PRECISION',
                           f'marker3_dist={dist:.3f}')
            self._cancel_current_goal()

        elif action == 'homing_enter':
            self.get_logger().info(
                f'Marker {marker_id} at {dist:.2f} m — VISUAL_HOMING '
                f'(marker odom ≈ {homing_ox:.2f},{homing_oy:.2f})'
            )
            self.log_event('STATE_CHANGE', 'Transitioned to VISUAL_HOMING',
                           f'marker{marker_id}_dist={dist:.3f} '
                           f'odom=({homing_ox:.2f},{homing_oy:.2f})')
            self._cancel_current_goal()
            self._fire_visual_homing_goal(homing_ox, homing_oy)

        elif action == 'buffering_direct':
            self.get_logger().info(
                f'Marker {marker_id} at {dist:.2f} m ≤ {self.VISUAL_HOMING_CLOSE_M:.2f} m '
                '— BUFFERING_ROUGH (direct, already close)'
            )
            self.log_event('STATE_CHANGE', 'Transitioned to BUFFERING_ROUGH',
                           f'marker{marker_id}_dist={dist:.3f} (direct)')
            self._cancel_current_goal()

        elif action == 'homing_exit':
            self.get_logger().info(
                f'Visual homing: marker {marker_id} now at {dist:.2f} m '
                '— BUFFERING_ROUGH'
            )
            self.log_event('STATE_CHANGE', 'Transitioned to BUFFERING_ROUGH',
                           f'marker{marker_id}_dist={dist:.3f} (homing exit)')
            self._cancel_current_goal()

    # ── RViz goal_pose proxy ──────────────────────────────────────────────────

    def _on_goal_pose(self, msg: PoseStamped):
        """Intercepts RViz 2D Nav Goal.

        IDLE      → transition to COMMUTING, send zone nav goal (normal start).
        COMMUTING → preempt current zone drive, re-target to new clicked point.
        All else  → ignore (active docking or terminal state).
        """
        with self._lock:
            state = self._state
            if state == DockState.IDLE:
                self._state = DockState.COMMUTING
                self._rough_buf.clear()
                self._prec_buf.clear()
                do_cancel = False
            elif state == DockState.COMMUTING:
                self._rough_buf.clear()
                self._prec_buf.clear()
                do_cancel = True
            else:
                self.get_logger().info(
                    f'/goal_pose ignored — state is {state.name}'
                )
                return

        if do_cancel:
            self._cancel_current_goal()
            self.get_logger().info(
                f'New /goal_pose — re-targeting zone '
                f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
            )
        else:
            # Fresh run starting from IDLE
            self._mission_logged = False
            self._run_start_time = time.time()
            self.get_logger().info(
                f'IDLE → COMMUTING — navigating to zone '
                f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}); '
                'spiral triggers on arrival if no tag detected'
            )

        self.log_event(
            'GOAL_POSE', 'RViz zone goal received',
            f'({msg.pose.position.x:.2f},{msg.pose.position.y:.2f})',
        )

        self._set_speed_limit(1.0)   # ensure full speed during zone commute
        msg.header.stamp = self.get_clock().now().to_msg()
        self._send_nav_goal(msg, self._zone_nav_done_cb)

    # ── odom callback — overshoot kill-switch ────────────────────────────────

    def _on_odom(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q  = msg.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Always record last known pose (used for final combined error)
        self._last_odom_pose = (px, py, yaw)

        # Overshoot monitor is only active during final dock approach
        with self._lock:
            if self._state != DockState.NAV_TO_DOCK:
                return
            gx, gy, _ = self._reference_dock_pose

        current_dist = math.sqrt((px - gx)**2 + (py - gy)**2)

        should_abort = False
        abort_reason = ''
        with self._lock:
            if self._state != DockState.NAV_TO_DOCK:
                return

            # Arm only when robot enters the 10 cm danger zone
            if current_dist <= self.OVERSHOOT_ARM_M:
                self.min_distance_to_goal = min(self.min_distance_to_goal, current_dist)

            # Trigger: armed AND robot is retreating more than 2 cm from its closest point
            if (self.min_distance_to_goal <= self.OVERSHOOT_ARM_M
                    and current_dist > self.min_distance_to_goal + self.OVERSHOOT_DELTA_M):
                self._state   = DockState.ABORTED
                should_abort  = True
                abort_reason  = (
                    f'Goal overshoot detected inside danger zone. '
                    f'Min dist: {self.min_distance_to_goal:.3f}, '
                    f'Current: {current_dist:.3f}'
                )

        if should_abort:
            self.get_logger().error(f'Overshoot kill-switch: {abort_reason}')
            self.log_event('SAFETY_ABORT', abort_reason)
            self._stop_robot()
            self._cancel_current_goal()
            self._restore_commute_config()
            self._set_speed_limit(1.0)
            self._log_mission_end('ABORTED_OVERSHOOT')

    # ── visual homing ─────────────────────────────────────────────────────────

    def _fire_visual_homing_goal(self, ox: float, oy: float):
        """Navigate toward the AprilTag marker's odom position (ox, oy).

        The nav goal is placed VISUAL_HOMING_NAV_STANDOFF_M short of the marker
        so it stays in free space.  _on_any_marker cancels this goal early once
        the camera-frame distance drops to VISUAL_HOMING_CLOSE_M — the nav goal
        just gives Nav2 a heading to follow.
        """
        rx, ry, _ = self._last_odom_pose
        approach_angle = math.atan2(oy - ry, ox - rx)

        hx   = ox - self.VISUAL_HOMING_NAV_STANDOFF_M * math.cos(approach_angle)
        hy   = oy - self.VISUAL_HOMING_NAV_STANDOFF_M * math.sin(approach_angle)
        hyaw = approach_angle   # face the marker

        goal_pose = _pose_stamped(hx, hy, hyaw, 'odom')
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f'Visual homing nav → ({hx:.3f}, {hy:.3f}) yaw={math.degrees(hyaw):.1f}°  '
            f'marker_odom=({ox:.3f},{oy:.3f})'
        )
        self.log_event(
            'GOAL_SENT', 'Visual homing nav goal sent',
            f'standoff=({hx:.3f},{hy:.3f}) approach_deg={math.degrees(approach_angle):.1f}',
        )
        self._set_speed_limit(1.0)
        self._send_nav_goal(goal_pose, self._visual_homing_done_cb)

    def _visual_homing_done_cb(self, future):
        """Fires when the visual-homing nav goal finishes.

        Success → robot is at the close-range standoff; start BUFFERING_ROUGH.
        Failure → fall back to LOCAL_SEARCH spiral (box was detected, so we
                  know roughly where it is; the spiral will re-find it).
        """
        result = future.result()
        status = result.status if result else GoalStatus.STATUS_ABORTED

        with self._lock:
            state = self._state

        # Already transitioned out by _on_box_pose close-enough check
        # or by the precision trigger — nothing to do.
        if state != DockState.VISUAL_HOMING:
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                'Visual homing goal reached — starting BUFFERING_ROUGH from close range'
            )
            self.log_event(
                'STATE_CHANGE', 'Transitioned to BUFFERING_ROUGH',
                'visual homing nav goal succeeded',
            )
            with self._lock:
                self._state = DockState.BUFFERING_ROUGH
                self._rough_buf.clear()
            # Buffer will be filled by the next _on_box_pose callbacks
        else:
            self.get_logger().warn(
                f'Visual homing failed (status={status}) — starting LOCAL_SEARCH spiral'
            )
            self.log_event(
                'STATE_CHANGE', 'Transitioned to LOCAL_SEARCH',
                f'visual homing failed status={status}',
            )
            with self._lock:
                self._state = DockState.LOCAL_SEARCH
            self._fire_local_search()

    # ── goal firing ───────────────────────────────────────────────────────────

    def _fire_staging_goal(self, buf: list[tuple[float, float, float]]):
        xs   = [p[0] for p in buf]
        ys   = [p[1] for p in buf]
        yaws = [p[2] for p in buf]

        avg_x   = _outlier_mean(xs)
        avg_y   = _outlier_mean(ys)
        avg_yaw = _circular_mean(yaws)

        sx   = avg_x - self.APPROACH_DIST_ROUGH * math.cos(avg_yaw)
        sy   = avg_y - self.APPROACH_DIST_ROUGH * math.sin(avg_yaw)
        syaw = avg_yaw

        goal_pose = _pose_stamped(sx, sy, syaw, 'odom')
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f'Staging → ({sx:.3f}, {sy:.3f}) yaw={math.degrees(syaw):.1f}°'
        )
        self.log_event(
            'STATE_CHANGE', 'Transitioned to NAV_TO_STAGING',
            f'goal=({sx:.3f},{sy:.3f}) yaw={math.degrees(syaw):.1f}deg',
        )
        self._set_speed_limit(1.0)
        self._send_nav_goal(goal_pose, self._staging_done_cb)

    def _fire_dock_goal(self, buf: list[tuple[float, float, float]]):
        xs   = [p[0] for p in buf]
        ys   = [p[1] for p in buf]
        yaws = [p[2] for p in buf]

        avg_x   = _outlier_mean(xs)
        avg_y   = _outlier_mean(ys)
        avg_yaw = _circular_mean(yaws)

        with self._lock:
            self._reference_dock_pose = (avg_x, avg_y, avg_yaw)
            self.min_distance_to_goal = float('inf')  # reset watermark for fresh approach

        goal_pose = _pose_stamped(avg_x, avg_y, avg_yaw, 'odom')
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f'Dock → ({avg_x:.3f}, {avg_y:.3f}) yaw={math.degrees(avg_yaw):.1f}°'
        )
        self.log_event(
            'STATE_CHANGE', 'Transitioned to NAV_TO_DOCK',
            f'goal=({avg_x:.3f},{avg_y:.3f}) yaw={math.degrees(avg_yaw):.1f}deg',
        )
        self._set_speed_limit(self.DOCK_SPEED_LIMIT)
        self._configure_precision_approach()   # footprint + inflation + velocity (blocking)
        self._send_nav_goal(goal_pose, self._dock_done_cb)

    # ── Nav2 NavigateToPose helpers ───────────────────────────────────────────

    def _send_nav_goal(self, pose: PoseStamped, done_cb):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available — back to IDLE')
            self.log_event('ERROR', 'Nav2 action server unavailable')
            with self._lock:
                self._state = DockState.IDLE
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._goal_accepted_cb(f, done_cb))

    def _goal_accepted_cb(self, future, done_cb):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Nav2 rejected goal — back to IDLE')
            self.log_event('ERROR', 'Nav2 goal rejected')
            with self._lock:
                self._state = DockState.IDLE
            return

        self.log_event('GOAL_ACCEPTED', 'Nav2 goal accepted and tracking started')
        with self._lock:
            self._current_goal_handle = handle
        handle.get_result_async().add_done_callback(done_cb)

    def _cancel_current_goal(self):
        with self._lock:
            handle = self._current_goal_handle
        if handle is not None:
            handle.cancel_goal_async()

    def _staging_done_cb(self, future):
        result = future.result()
        status = result.status if result else GoalStatus.STATUS_ABORTED

        with self._lock:
            state = self._state

        if state == DockState.BUFFERING_PRECISION:
            return  # intentionally cancelled by marker-3 proximity trigger

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Staging failed (status={status}) — IDLE')
            self.log_event(
                'STATE_CHANGE', 'Transitioned to IDLE',
                f'staging_failed status={status}',
            )
            with self._lock:
                self._state = DockState.IDLE
        else:
            # Robot is stationary at staging — if marker 3 is already visible,
            # the proximity trigger will never fire on its own. Auto-proceed.
            now_ns = self.get_clock().now().nanoseconds
            m3 = self._marker_seen.get(3)
            if m3 and (now_ns - m3[0]) / 1e9 <= 2.0:
                with self._lock:
                    if self._state == DockState.NAV_TO_STAGING:
                        self._state = DockState.BUFFERING_PRECISION
                        self._prec_buf.clear()
                self.get_logger().info(
                    f'Staging reached — Marker 3 visible at {m3[1]:.2f} m, '
                    'auto-triggering BUFFERING_PRECISION'
                )
                self.log_event(
                    'STATE_CHANGE', 'Transitioned to BUFFERING_PRECISION',
                    f'auto-triggered on staging arrival, marker3_dist={m3[1]:.3f}',
                )
            else:
                self.get_logger().info('Staging reached — awaiting marker-3 close trigger')

    def _dock_done_cb(self, future):
        result = future.result()
        status = result.status if result else GoalStatus.STATUS_ABORTED

        with self._lock:
            state = self._state

        if state == DockState.ABORTED:
            return  # overshoot kill-switch already handled this

        if status == GoalStatus.STATUS_SUCCEEDED:
            rx, ry, ryaw = self._last_odom_pose
            gx, gy, gyaw = self._reference_dock_pose
            dx   = rx - gx
            dy   = ry - gy
            dyaw = _wrap_pi(ryaw - gyaw)
            L    = 0.20
            combined_error = math.sqrt(dx**2 + dy**2 + (L * dyaw)**2)

            self.log_event(
                'MISSION_SUCCESS', 'Final Combined Error',
                f'{combined_error:.6f}',
            )
            with self._lock:
                self._state = DockState.LOCKED
            self._stop_robot()
            self.get_logger().info(
                f'DOCKED — state LOCKED  combined_error={combined_error:.4f} m'
            )
            self._log_mission_end('SUCCEEDED')
        else:
            self.get_logger().warn(f'Dock failed (status={status}) — IDLE')
            self.log_event(
                'STATE_CHANGE', 'Transitioned to IDLE',
                f'dock_failed status={status}',
            )
            self._restore_commute_config()
            self._set_speed_limit(1.0)
            with self._lock:
                self._state = DockState.IDLE
            self._log_mission_end('FAILED_NAV2')

    # ── speed / footprint helpers ─────────────────────────────────────────────

    def _set_speed_limit(self, speed: float):
        msg = SpeedLimit()
        msg.percentage  = False
        msg.speed_limit = speed
        self._speed_pub.publish(msg)

    def _configure_precision_approach(self):
        """Set 10×10 cm footprint, tight inflation, and slow velocity before NAV_TO_DOCK.

        Blocks until each service call completes (or times out). Safe to call
        from cb_sub (ReentrantCallbackGroup) — service responses are dispatched
        through cb_srv which is a different group, so no deadlock.
        """
        def _wait(future, label: str, timeout: float = 3.0):
            deadline = time.time() + timeout
            while not future.done():
                if time.time() > deadline:
                    self.get_logger().warn(f'{label} timed out — continuing anyway')
                    return
                time.sleep(0.01)

        def _fp_req(fp_str: str) -> SetParameters.Request:
            pv = ParameterValue()
            pv.type         = ParameterType.PARAMETER_STRING
            pv.string_value = fp_str
            p = Parameter()
            p.name  = 'footprint'
            p.value = pv
            req = SetParameters.Request()
            req.parameters = [p]
            return req

        def _inflation_req(radius: float, scale: float) -> SetParameters.Request:
            req = SetParameters.Request()
            for name, val in [
                ('inflation_layer.inflation_radius',    radius),
                ('inflation_layer.cost_scaling_factor', scale),
            ]:
                pv = ParameterValue()
                pv.type         = ParameterType.PARAMETER_DOUBLE
                pv.double_value = val
                p = Parameter()
                p.name = name; p.value = pv
                req.parameters.append(p)
            return req

        def _velocity_req(max_vel: list) -> SetParameters.Request:
            pv = ParameterValue()
            pv.type              = ParameterType.PARAMETER_DOUBLE_ARRAY
            pv.double_array_value = max_vel
            p = Parameter()
            p.name = 'max_velocity'; p.value = pv
            req = SetParameters.Request()
            req.parameters = [p]
            return req

        # 1. Footprint 10×10 cm on both costmaps
        for client, label in [
            (self._fp_client,        'local_costmap/footprint'),
            (self._fp_global_client, 'global_costmap/footprint'),
        ]:
            if client.wait_for_service(timeout_sec=2.0):
                _wait(client.call_async(_fp_req(self._FP_DOCK)), label)
            else:
                self.get_logger().warn(f'{label} unavailable — skipping')

        self.get_logger().info('Precision footprint set: 10×10 cm')

        # 2. Inflation 0.11 m / 100.0 on local costmap
        if self._fp_client.wait_for_service(timeout_sec=2.0):
            _wait(
                self._fp_client.call_async(
                    _inflation_req(self.INFLATION_RADIUS_PRECISION,
                                   self.INFLATION_SCALE_PRECISION)
                ),
                'local_costmap/inflation',
            )
            self.get_logger().info(
                f'Precision inflation set: radius={self.INFLATION_RADIUS_PRECISION} m, '
                f'scale={self.INFLATION_SCALE_PRECISION}'
            )

        # 3. Velocity smoother cap 0.2 m/s
        if self._velocity_smoother_client.wait_for_service(timeout_sec=2.0):
            _wait(
                self._velocity_smoother_client.call_async(
                    _velocity_req([0.2, 0.0, 5.0])
                ),
                'velocity_smoother/max_velocity',
            )
            self.get_logger().info('Velocity smoother capped at 0.2 m/s')
        else:
            self.get_logger().warn('velocity_smoother/set_parameters unavailable — skipping')

        # 4. Flush local costmap
        if self._clear_local_client.wait_for_service(timeout_sec=2.0):
            _wait(
                self._clear_local_client.call_async(Empty.Request()),
                'clear_local_costmap',
            )
            self.get_logger().info('Local costmap flushed — transitioning to NAV_TO_DOCK')

    def _restore_commute_config(self):
        """Restore footprint, inflation, and velocity to normal commute values.

        Always fire-and-forget (no blocking waits). Safe to call from cb_srv
        (_on_reset) where blocking would deadlock, and from cb_sub / cb_nav
        where we simply don't need to wait.
        """
        def _fp_req(fp_str: str) -> SetParameters.Request:
            pv = ParameterValue()
            pv.type         = ParameterType.PARAMETER_STRING
            pv.string_value = fp_str
            p = Parameter()
            p.name = 'footprint'; p.value = pv
            req = SetParameters.Request()
            req.parameters = [p]
            return req

        def _inflation_req(radius: float, scale: float) -> SetParameters.Request:
            req = SetParameters.Request()
            for name, val in [
                ('inflation_layer.inflation_radius',    radius),
                ('inflation_layer.cost_scaling_factor', scale),
            ]:
                pv = ParameterValue()
                pv.type         = ParameterType.PARAMETER_DOUBLE
                pv.double_value = val
                p = Parameter()
                p.name = name; p.value = pv
                req.parameters.append(p)
            return req

        def _velocity_req(max_vel: list) -> SetParameters.Request:
            pv = ParameterValue()
            pv.type               = ParameterType.PARAMETER_DOUBLE_ARRAY
            pv.double_array_value = max_vel
            p = Parameter()
            p.name = 'max_velocity'; p.value = pv
            req = SetParameters.Request()
            req.parameters = [p]
            return req

        fp_r = _fp_req(self._FP_NORMAL)
        inf_r = _inflation_req(self.INFLATION_RADIUS_NORMAL, self.INFLATION_SCALE_NORMAL)
        vel_r = _velocity_req([0.5, 0.0, 5.0])

        if self._fp_client.service_is_ready():
            self._fp_client.call_async(fp_r)
        if self._fp_global_client.service_is_ready():
            self._fp_global_client.call_async(_fp_req(self._FP_NORMAL))
        if self._fp_client.service_is_ready():
            self._fp_client.call_async(inf_r)
        if self._velocity_smoother_client.service_is_ready():
            self._velocity_smoother_client.call_async(vel_r)
        if self._clear_local_client.service_is_ready():
            self._clear_local_client.call_async(Empty.Request())
        if self._clear_global_client.service_is_ready():
            self._clear_global_client.call_async(Empty.Request())

        self.get_logger().info(
            f'Commute config restored: footprint={self._FP_NORMAL}, '
            f'inflation={self.INFLATION_RADIUS_NORMAL} m, velocity_smoother=0.5 m/s'
        )

    def _stop_robot(self):
        self._cmd_pub.publish(Twist())

    # ── reset service ─────────────────────────────────────────────────────────

    def _on_reset(self, _req, resp):
        self.get_logger().info('Reset requested')
        self.log_event('RESET', 'Mission reset triggered by service call')

        with self._lock:
            pre_reset_state = self._state

        self._cancel_current_goal()
        self._stop_robot()
        self._restore_commute_config()   # fire-and-forget: footprint + inflation + velocity
        self._set_speed_limit(1.0)

        # Log mission end based on what state we were in before reset
        if pre_reset_state == DockState.LOCKED:
            self._log_mission_end('SUCCEEDED')
        elif pre_reset_state == DockState.ABORTED:
            self._log_mission_end('ABORTED')
        elif pre_reset_state not in (DockState.IDLE,):
            self._log_mission_end('CANCELLED')

        with self._lock:
            self._state               = DockState.IDLE
            self._rough_buf.clear()
            self._prec_buf.clear()
            self.min_distance_to_goal = float('inf')
            self._current_goal_handle = None
        self.log_event('STATE_CHANGE', 'Transitioned to IDLE', 'via reset')
        resp.success = True
        resp.message = 'Reset to IDLE'
        return resp


def main():
    rclpy.init()
    node = DockingSupervisor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
