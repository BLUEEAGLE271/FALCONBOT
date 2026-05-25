#!/usr/bin/env python3
"""
docking_supervisor.py — AprilTag Docking State Machine
=======================================================
Subscribes to /box_center_pose (from box_estimator) and drives the robot
through a two-phase dock sequence:

  SEARCHING           → waiting for any /box_center_pose sample
  LOCAL_SEARCH        → Archimedean spiral sweep (NavigateThroughPoses)
  BUFFERING_ROUGH     → collecting BUF_ROUGH samples to compute staging goal
  NAV_TO_STAGING      → Nav2 driving to 0.5 m standoff pose
  BUFFERING_PRECISION → marker-3 seen close (<0.6 m); collecting precision samples
  NAV_TO_DOCK         → Nav2 driving to exact box centre with shrunken footprint
  LOCKED              → docked, final combined error logged
  ABORTED             → CTE kill-switch or spiral failure; awaiting reset

Thesis extras (non-breaking):
  • CSV logger  → ~/ros2_ws/src/parking/results/thesis_docking_metrics.csv
  • CTE kill-switch (NAV_TO_DOCK only): abort if CTE > 0.05 m OR 5-tick rise
  • Final combined error logged to CSV on successful dock
  • Expanding Archimedean spiral (NavigateThroughPoses) if no tag in SEARCH_TIMEOUT_S

Reset via: ros2 run parking reset_mission
       or: ros2 service call /docking_supervisor/reset_mission std_srvs/srv/Trigger
"""

import csv
import math
import os
import threading
from datetime import datetime
from enum import Enum, auto

import tf_transformations as tft
import rclpy
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
from std_srvs.srv import Trigger


class DockState(Enum):
    SEARCHING           = auto()
    LOCAL_SEARCH        = auto()   # expanding spiral sweep
    BUFFERING_ROUGH     = auto()
    NAV_TO_STAGING      = auto()
    BUFFERING_PRECISION = auto()
    NAV_TO_DOCK         = auto()
    LOCKED              = auto()
    ABORTED             = auto()   # CTE kill-switch or spiral exhausted


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


def _perp_dist(px: float, py: float,
               ax: float, ay: float,
               bx: float, by: float) -> float:
    """Perpendicular distance from point P to infinite line through A→B."""
    dx, dy = bx - ax, by - ay
    len_sq = dx*dx + dy*dy
    if len_sq < 1e-9:
        return math.sqrt((px - ax)**2 + (py - ay)**2)
    return abs((py - ay)*dx - (px - ax)*dy) / math.sqrt(len_sq)


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
    PROXIMITY_TRIGGER_M  = 0.60   # m — marker-3 camera distance triggers precision
    DOCK_SPEED_LIMIT     = 0.10   # m/s — speed cap during final approach
    CTE_LIMIT_M          = 0.05   # m — CTE threshold for kill-switch
    CTE_RISE_TICKS       = 5      # consecutive rising CTE ticks → kill
    SEARCH_TIMEOUT_S     = 20.0   # s  — SEARCHING with no tag → spiral sweep

    _FP_DOCK   = '[[-0.09,-0.09],[0.09,-0.09],[0.09,0.09],[-0.09,0.09]]'
    _FP_NORMAL = '[[-0.13,-0.13],[0.13,-0.13],[0.13,0.13],[-0.13,0.13]]'

    _CSV_PATH  = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'src', 'parking', 'results',
        'thesis_docking_metrics.csv',
    )

    def __init__(self):
        super().__init__('docking_supervisor')

        # ── state machine ────────────────────────────────────────────────────
        self._state = DockState.SEARCHING
        self._lock  = threading.Lock()

        self._rough_buf: list[tuple[float, float, float]] = []
        self._prec_buf:  list[tuple[float, float, float]] = []
        self._current_goal_handle = None
        self._footprint_shrunken  = False

        # ── thesis data logger ───────────────────────────────────────────────
        self._csv_lock = threading.Lock()
        self._init_csv()

        # ── CTE kill-switch state ────────────────────────────────────────────
        self._staging_xy:          tuple[float, float]        = (0.0, 0.0)
        self._dock_xy:             tuple[float, float]        = (0.0, 0.0)
        self._reference_dock_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_odom_pose:      tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._cte_history:         list[float]                = []

        # ── LOCAL_SEARCH timing ──────────────────────────────────────────────
        # 0 = "not yet started timing"; set by _check_search_timeout on entry
        self._search_start_ns: int = 0

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
        self.create_subscription(
            PoseStamped, '/apriltag/marker_3',
            self._on_marker3, be_qos, callback_group=cb_sub,
        )
        self.create_subscription(
            Odometry, '/odom',
            self._on_odom, 10, callback_group=cb_sub,
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

        # ── parameter client (footprint) ─────────────────────────────────────
        self._fp_client = self.create_client(
            SetParameters, '/local_costmap/local_costmap/set_parameters',
            callback_group=cb_srv,
        )

        # ── search timeout polling timer (1 Hz) ──────────────────────────────
        self.create_timer(1.0, self._check_search_timeout, callback_group=cb_sub)

        # ── reset service ────────────────────────────────────────────────────
        self.create_service(
            Trigger, '~/reset_mission',
            self._on_reset, callback_group=cb_srv,
        )

        self.log_event('NODE_START', 'DockingSupervisor initialised')
        self.get_logger().info('DockingSupervisor ready — SEARCHING')

    # ── CSV logger ────────────────────────────────────────────────────────────

    def _init_csv(self):
        os.makedirs(os.path.dirname(self._CSV_PATH), exist_ok=True)
        write_header = (
            not os.path.exists(self._CSV_PATH)
            or os.path.getsize(self._CSV_PATH) == 0
        )
        with self._csv_lock:
            with open(self._CSV_PATH, 'a', newline='') as f:
                if write_header:
                    csv.writer(f).writerow(
                        ['timestamp_iso', 'event_type', 'description', 'value']
                    )

    def log_event(self, event_type: str, description: str, value: str = ''):
        now = datetime.now().strftime('%Y-%m-%dT%H:%M:%S.%f')
        with self._csv_lock:
            with open(self._CSV_PATH, 'a', newline='') as f:
                csv.writer(f).writerow([now, event_type, description, value])

    # ── search timeout → LOCAL_SEARCH trigger ─────────────────────────────────

    def _check_search_timeout(self):
        """1 Hz timer. Starts LOCAL_SEARCH if stuck in SEARCHING too long."""
        should_spiral = False
        with self._lock:
            if self._state != DockState.SEARCHING:
                self._search_start_ns = 0   # reset so re-entry gets fresh timing
                return
            now_ns = self.get_clock().now().nanoseconds
            if self._search_start_ns == 0:
                self._search_start_ns = now_ns
                return
            elapsed = (now_ns - self._search_start_ns) / 1e9
            if elapsed >= self.SEARCH_TIMEOUT_S:
                self._state           = DockState.LOCAL_SEARCH
                self._search_start_ns = 0
                should_spiral         = True

        if should_spiral:
            self.get_logger().info(
                f'No tag in {self.SEARCH_TIMEOUT_S:.0f}s — starting LOCAL_SEARCH spiral'
            )
            self._fire_local_search()

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
            self.get_logger().error('Spiral goal rejected — back to SEARCHING')
            self.log_event('ERROR', 'NavigateThroughPoses goal rejected')
            with self._lock:
                self._state           = DockState.SEARCHING
                self._search_start_ns = 0
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

        # Tag was detected mid-spiral: state already changed to BUFFERING_*
        # by _on_box_pose or _on_marker3 — nothing left to do here.
        if state in (DockState.BUFFERING_ROUGH, DockState.BUFFERING_PRECISION):
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
            # Cancelled for an unexpected reason — retry from SEARCHING
            self.get_logger().warn(
                f'Spiral ended unexpectedly (status={status}) — SEARCHING'
            )
            self.log_event(
                'STATE_CHANGE', 'Transitioned to SEARCHING',
                f'spiral_status={status}',
            )
            with self._lock:
                self._state           = DockState.SEARCHING
                self._search_start_ns = 0

    # ── box_center_pose callback ───────────────────────────────────────────────

    def _on_box_pose(self, msg: PoseStamped):
        q = msg.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        sample = (msg.pose.position.x, msg.pose.position.y, yaw)

        fire_staging       = False
        fire_dock          = False
        cancel_spiral      = False

        with self._lock:
            if self._state == DockState.SEARCHING:
                self._rough_buf.clear()
                self._rough_buf.append(sample)
                self._state           = DockState.BUFFERING_ROUGH
                self._search_start_ns = 0          # stop timeout clock
                self.get_logger().info('Box detected — BUFFERING_ROUGH')
                self.log_event('STATE_CHANGE', 'Transitioned to BUFFERING_ROUGH')

            elif self._state == DockState.LOCAL_SEARCH:
                # Tag found during spiral — interrupt sweep, begin buffering
                self._rough_buf.clear()
                self._rough_buf.append(sample)
                self._state    = DockState.BUFFERING_ROUGH
                cancel_spiral  = True
                self.get_logger().info('Tag detected during spiral — BUFFERING_ROUGH')
                self.log_event(
                    'STATE_CHANGE', 'Transitioned to BUFFERING_ROUGH',
                    'tag detected during LOCAL_SEARCH',
                )

            elif self._state == DockState.BUFFERING_ROUGH:
                self._rough_buf.append(sample)
                if len(self._rough_buf) >= self.BUF_ROUGH:
                    self._state  = DockState.NAV_TO_STAGING
                    fire_staging = True

            elif self._state == DockState.BUFFERING_PRECISION:
                self._prec_buf.append(sample)
                if len(self._prec_buf) >= self.BUF_PRECISION:
                    self._state = DockState.NAV_TO_DOCK
                    fire_dock   = True

        if cancel_spiral:
            self._cancel_current_goal()

        if fire_staging:
            with self._lock:
                buf = list(self._rough_buf)
            self._fire_staging_goal(buf)

        if fire_dock:
            with self._lock:
                buf = list(self._prec_buf)
            self._fire_dock_goal(buf)

    # ── marker-3 proximity callback ────────────────────────────────────────────

    def _on_marker3(self, msg: PoseStamped):
        p = msg.pose.position
        dist = math.sqrt(p.x**2 + p.y**2 + p.z**2)

        should_cancel = False
        with self._lock:
            # Trigger from NAV_TO_STAGING (existing) OR LOCAL_SEARCH (new)
            if (self._state in (DockState.NAV_TO_STAGING, DockState.LOCAL_SEARCH)
                    and dist < self.PROXIMITY_TRIGGER_M):
                self._state   = DockState.BUFFERING_PRECISION
                self._prec_buf.clear()
                should_cancel = True

        if should_cancel:
            self.get_logger().info(
                f'Marker-3 at {dist:.2f} m — cancelling goal, BUFFERING_PRECISION'
            )
            self.log_event(
                'STATE_CHANGE', 'Transitioned to BUFFERING_PRECISION',
                f'marker3_dist={dist:.3f}',
            )
            self._cancel_current_goal()

    # ── odom callback — CTE kill-switch ───────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q  = msg.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Always record last known pose (used for final combined error)
        self._last_odom_pose = (px, py, yaw)

        # CTE monitoring is only active during final dock approach
        with self._lock:
            if self._state != DockState.NAV_TO_DOCK:
                return
            ax, ay = self._staging_xy
            bx, by = self._dock_xy

        cte = _perp_dist(px, py, ax, ay, bx, by)

        should_abort = False
        abort_reason = ''
        with self._lock:
            if self._state != DockState.NAV_TO_DOCK:
                return
            self._cte_history.append(cte)
            if len(self._cte_history) > self.CTE_RISE_TICKS:
                self._cte_history.pop(0)

            over_threshold = cte > self.CTE_LIMIT_M
            strictly_rising = (
                len(self._cte_history) == self.CTE_RISE_TICKS
                and all(
                    self._cte_history[i] < self._cte_history[i + 1]
                    for i in range(self.CTE_RISE_TICKS - 1)
                )
            )

            if over_threshold or strictly_rising:
                self._state = DockState.ABORTED
                self._cte_history.clear()
                should_abort = True
                abort_reason = (
                    f'CTE={cte:.4f}m > {self.CTE_LIMIT_M}m'
                    if over_threshold
                    else f'5-tick rising CTE, last={cte:.4f}m'
                )

        if should_abort:
            self.get_logger().error(f'CTE kill-switch: {abort_reason}')
            self.log_event('SAFETY_ABORT', 'CTE threshold exceeded', abort_reason)
            self._stop_robot()
            self._cancel_current_goal()
            self._restore_footprint()
            self._set_speed_limit(1.0)

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

        with self._lock:
            self._staging_xy = (sx, sy)

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
            self._dock_xy             = (avg_x, avg_y)
            self._reference_dock_pose = (avg_x, avg_y, avg_yaw)
            self._cte_history.clear()

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
        self._shrink_footprint()
        self._send_nav_goal(goal_pose, self._dock_done_cb)

    # ── Nav2 NavigateToPose helpers ───────────────────────────────────────────

    def _send_nav_goal(self, pose: PoseStamped, done_cb):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available — back to SEARCHING')
            self.log_event('ERROR', 'Nav2 action server unavailable')
            with self._lock:
                self._state = DockState.SEARCHING
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._goal_accepted_cb(f, done_cb))

    def _goal_accepted_cb(self, future, done_cb):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('Nav2 rejected goal — back to SEARCHING')
            self.log_event('ERROR', 'Nav2 goal rejected')
            with self._lock:
                self._state = DockState.SEARCHING
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
            self.get_logger().warn(f'Staging failed (status={status}) — SEARCHING')
            self.log_event(
                'STATE_CHANGE', 'Transitioned to SEARCHING',
                f'staging_failed status={status}',
            )
            with self._lock:
                self._state = DockState.SEARCHING
        else:
            self.get_logger().info('Staging reached — awaiting marker-3 close trigger')

    def _dock_done_cb(self, future):
        result = future.result()
        status = result.status if result else GoalStatus.STATUS_ABORTED

        with self._lock:
            state = self._state

        if state == DockState.ABORTED:
            return  # CTE kill-switch already handled this

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
        else:
            self.get_logger().warn(f'Dock failed (status={status}) — SEARCHING')
            self.log_event(
                'STATE_CHANGE', 'Transitioned to SEARCHING',
                f'dock_failed status={status}',
            )
            self._restore_footprint()
            self._set_speed_limit(1.0)
            with self._lock:
                self._state = DockState.SEARCHING

    # ── speed / footprint helpers ─────────────────────────────────────────────

    def _set_speed_limit(self, speed: float):
        msg = SpeedLimit()
        msg.percentage  = False
        msg.speed_limit = speed
        self._speed_pub.publish(msg)

    def _shrink_footprint(self):
        self._set_footprint(self._FP_DOCK)
        self._footprint_shrunken = True

    def _restore_footprint(self):
        if self._footprint_shrunken:
            self._set_footprint(self._FP_NORMAL)
            self._footprint_shrunken = False

    def _set_footprint(self, fp_str: str):
        if not self._fp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Footprint service unavailable — skipping')
            return
        pv = ParameterValue()
        pv.type         = ParameterType.PARAMETER_STRING
        pv.string_value = fp_str
        param = Parameter()
        param.name  = 'footprint'
        param.value = pv
        req = SetParameters.Request()
        req.parameters = [param]
        self._fp_client.call_async(req)

    def _stop_robot(self):
        self._cmd_pub.publish(Twist())

    # ── reset service ─────────────────────────────────────────────────────────

    def _on_reset(self, _req, resp):
        self.get_logger().info('Reset requested')
        self.log_event('RESET', 'Mission reset triggered by service call')
        self._cancel_current_goal()
        self._stop_robot()
        self._restore_footprint()
        self._set_speed_limit(1.0)
        with self._lock:
            self._state           = DockState.SEARCHING
            self._rough_buf.clear()
            self._prec_buf.clear()
            self._cte_history.clear()
            self._current_goal_handle = None
            self._search_start_ns     = 0     # restart search timeout clock
        self.log_event('STATE_CHANGE', 'Transitioned to SEARCHING', 'via reset')
        resp.success = True
        resp.message = 'Reset to SEARCHING'
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
