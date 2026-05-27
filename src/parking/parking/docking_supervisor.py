#!/usr/bin/env python3
"""
docking_supervisor.py — AprilTag Docking State Machine (RPP + Visual Servo)
============================================================================
Architecture
------------
Phase 1  COMMUTING      Regulated Pure Pursuit via Nav2 NavigateToPose
Phase 2  VISUAL_HOMING  RPP drives toward detected AprilTag (any marker)
Phase 3  BUFFERING_ROUGH / NAV_TO_STAGING  box_center_pose averaging → RPP staging
Phase 4  VISUAL_SERVO_DOCK  Direct P-controller on /cmd_vel from raw marker pose

State Machine
-------------
IDLE              waiting for RViz 2D Nav Goal
COMMUTING         RPP to zone; marker seen → VISUAL_HOMING
VISUAL_HOMING     RPP approaching tag; close (≤0.6 m) → BUFFERING_ROUGH
LOCAL_SEARCH      Archimedean spiral if no tag at zone
BUFFERING_ROUGH   10 box_center_pose samples → staging goal
NAV_TO_STAGING    RPP to staging; marker-3 close → VISUAL_SERVO_DOCK
VISUAL_SERVO_DOCK P-controller: vx=0.05, wz=clamp(1.5·marker_x, ±0.2)
DOCKED            terminal success
ABORTED           terminal failure (marker lost, spiral exhausted)

Reset: ros2 service call /docking_supervisor/reset_mission std_srvs/srv/Trigger
"""

import csv
import math
import os
import threading
import time
from datetime import datetime
from enum import Enum, auto

import rclpy
import rclpy.time
import tf_transformations as tft
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger


class DockState(Enum):
    IDLE              = auto()
    COMMUTING         = auto()
    VISUAL_HOMING     = auto()
    LOCAL_SEARCH      = auto()
    BUFFERING_ROUGH   = auto()
    NAV_TO_STAGING    = auto()
    VISUAL_SERVO_DOCK = auto()
    DOCKED            = auto()
    ABORTED           = auto()


# ── pure helpers ──────────────────────────────────────────────────────────────

def _circular_mean(angles: list[float]) -> float:
    return math.atan2(
        sum(math.sin(a) for a in angles),
        sum(math.cos(a) for a in angles),
    )


def _outlier_mean(vals: list[float]) -> float:
    if len(vals) < 6:
        return sum(vals) / len(vals)
    s = sorted(vals)
    return sum(s[2:-2]) / len(s[2:-2])


def _pose_stamped(x: float, y: float, yaw: float, frame: str = 'odom') -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = rclpy.time.Time().to_msg()   # zero stamp → use latest TF, no extrapolation
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
    R_MAX = 1.5
    REVS  = 2.0
    STEP  = math.pi / 4
    total = 2.0 * math.pi * REVS
    a     = R_MAX / total

    waypoints: list[PoseStamped] = []
    theta = STEP
    while theta <= total + 1e-9:
        r   = a * theta
        x   = x0 + r * math.cos(theta)
        y   = y0 + r * math.sin(theta)
        tx  = a * math.cos(theta) - r * math.sin(theta)
        ty  = a * math.sin(theta) + r * math.cos(theta)
        yaw = math.atan2(ty, tx)
        waypoints.append(_pose_stamped(x, y, yaw, 'odom'))
        theta += STEP
    return waypoints


# ── node ──────────────────────────────────────────────────────────────────────

class DockingSupervisor(Node):

    # Tuneable constants
    APPROACH_DIST_ROUGH          = 0.50   # m — staging standoff from box centre
    BUF_ROUGH                    = 10     # samples before firing staging goal
    PROXIMITY_TRIGGER_M          = 0.60   # m — marker-3 camera distance → VISUAL_SERVO_DOCK
    VISUAL_HOMING_CLOSE_M        = 0.60   # m — camera distance to exit visual homing
    VISUAL_HOMING_NAV_STANDOFF_M = 0.30   # m — nav goal standoff from marker (odom frame)
    BOX_LENGTH                   = 0.30   # m — dock condition: marker_z ≤ BOX_LENGTH / 2

    # Visual servo — Pure Pursuit + blind dead-reckoning
    BLIND_APPROACH_THRESHOLD     = 0.35   # m — switch to encoder dead-reckoning below this
    TARGET_STANDOFF              = 0.20   # m — stop this far from the tag face
    HARDWARE_BIAS_X              = 0.00   # m — camera lateral offset (camera at y=0 in base_link)
    LOOKAHEAD_DIST               = 0.15   # m — pure pursuit carrot projection distance
    K_STEER                      = 1.5    # rad/s per rad — steering gain
    FRICTION_DEADBAND            = 0.0   # rad/s — minimum wz to overcome static friction
    MAX_WZ                       = 3.5    # rad/s — angular velocity clamp
    V_LINEAR                     = 0.2   # m/s  — constant forward speed during servo

    _CSV_DIR = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'parking', 'results')

    def __init__(self):
        super().__init__('docking_supervisor')

        # ── state machine ────────────────────────────────────────────────────
        self._state = DockState.IDLE
        self._lock  = threading.Lock()
        self._rough_buf: list[tuple[float, float, float]] = []
        self._current_goal_handle = None

        # ── visual servo state ───────────────────────────────────────────────
        self._last_servo_marker_x:       float = 0.0
        self._last_servo_marker_z:       float = 1.0
        self._last_servo_detection_time: float = 0.0
        self._last_servo_marker_yaw:     float = 0.0
        self._last_servo_tag_odom:       tuple[float, float] = (0.0, 0.0)  # tag in odom frame
        self._last_servo_dock_yaw:       float        = 0.0   # dock Z-axis direction in odom
        self._last_servo_z_base:         tuple        = (1.0, 0.0)  # approach dir (ax,ay) in base_link
        self._servo_approach_yaw:        float | None = None   # frozen at first servo tick
        self._servo_tag_odom_frozen:     tuple | None = None   # frozen at first servo tick
        # blind dead-reckoning state
        self.in_blind_approach: bool       = False
        self.blind_start_pose:  tuple | None = None
        self.blind_target_dist: float      = 0.0

        # ── dynamic tuning parameters ─────────────────────────────────────────
        self.declare_parameter('lookahead_dist',          0.30)
        self.declare_parameter('k_steer',                 1.5) # pure-pursuit scale (1.0 = textbook)
        self.declare_parameter('friction_deadband',       0.0)
        self.declare_parameter('max_wz',                  3.5)
        self.declare_parameter('v_linear',                0.20)
        self.declare_parameter('target_standoff',         0.20)
        self.declare_parameter('blind_approach_threshold', 0.35)
        self.declare_parameter('align_threshold_deg',     12.0)

        self.lookahead_dist           = self.get_parameter('lookahead_dist').value
        self.k_steer                  = self.get_parameter('k_steer').value
        self.friction_deadband        = self.get_parameter('friction_deadband').value
        self.max_wz                   = self.get_parameter('max_wz').value
        self.v_linear                 = self.get_parameter('v_linear').value
        self.target_standoff          = self.get_parameter('target_standoff').value
        self.blind_approach_threshold = self.get_parameter('blind_approach_threshold').value
        self.align_threshold_deg      = self.get_parameter('align_threshold_deg').value

        self.add_on_set_parameters_callback(self._parameter_callback)

        # ── thesis data logger ───────────────────────────────────────────────
        self._csv_lock = threading.Lock()
        self._run_id   = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._CSV_PATH         = os.path.join(self._CSV_DIR, f'docking_events_{self._run_id}.csv')
        self._summary_CSV_PATH = os.path.join(self._CSV_DIR, 'docking_summary.csv')
        self._servo_debug_CSV: str = ''  # set fresh each time servo is entered
        self._init_csv()

        # ── mission tracking ─────────────────────────────────────────────────
        self._mission_logged      = False
        self._run_start_time      = time.time()
        self._reference_dock_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_odom_pose:      tuple[float, float, float] = (0.0, 0.0, 0.0)

        # ── cmd_vel monitor ──────────────────────────────────────────────────
        self.latest_vx: float = 0.0
        self.latest_wz: float = 0.0

        # ── per-marker debug telemetry ───────────────────────────────────────
        self._marker_seen: dict[int, tuple[int, float]] = {}

        # ── callback groups ──────────────────────────────────────────────────
        cb_sub    = ReentrantCallbackGroup()
        cb_nav    = MutuallyExclusiveCallbackGroup()
        cb_spiral = MutuallyExclusiveCallbackGroup()
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
        self.create_subscription(Odometry,    '/odom',      self._on_odom,     10, callback_group=cb_sub)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal_pose, 10, callback_group=cb_sub)
        self.create_subscription(Twist,       '/cmd_vel',   self._on_cmd_vel,  10, callback_group=cb_sub)

        # ── publishers ───────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── action clients ───────────────────────────────────────────────────
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=cb_nav,
        )
        self._spiral_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses', callback_group=cb_spiral,
        )

        # ── timers ───────────────────────────────────────────────────────────
        self.create_timer(1.0, self._debug_marker_cb,    callback_group=cb_sub)
        self._servo_timer = self.create_timer(0.1, self._servo_control_loop, callback_group=cb_sub)

        # ── reset service ────────────────────────────────────────────────────
        self.create_service(Trigger, '~/reset_mission', self._on_reset, callback_group=cb_srv)

        self.log_event('NODE_START', 'DockingSupervisor (RPP + Visual Servo) initialised')
        self.get_logger().info('DockingSupervisor ready — IDLE  (send RViz 2D Nav Goal to begin)')

    # ── 1 Hz debug ───────────────────────────────────────────────────────────

    def _debug_marker_cb(self):
        now_ns = self.get_clock().now().nanoseconds
        visible = [
            f'Marker {mid} ({dist:.2f} m)'
            for mid, (t_ns, dist) in self._marker_seen.items()
            if (now_ns - t_ns) / 1e9 <= 1.0
        ]
        if visible:
            self.get_logger().info('[DEBUG] VISIBLE: ' + '  |  '.join(visible))

    def _on_cmd_vel(self, msg: Twist):
        self.latest_vx = msg.linear.x
        self.latest_wz = msg.angular.z

    # ── end-of-mission summary ────────────────────────────────────────────────

    def _log_mission_end(self, result: str):
        if self._mission_logged:
            return
        self._mission_logged = True

        rx, ry, ryaw = self._last_odom_pose
        gx, gy, gyaw = self._reference_dock_pose
        got_dock_goal = (gx, gy) != (0.0, 0.0)

        if got_dock_goal:
            pose_err = math.sqrt((rx - gx) ** 2 + (ry - gy) ** 2)
            yaw_err  = abs(_wrap_pi(ryaw - gyaw))
        else:
            pose_err = yaw_err = float('nan')

        self.get_logger().info(
            f'MISSION END | Result: {result} | '
            f'Pose Error: {pose_err:.2f} m | Yaw Error: {yaw_err:.2f} rad'
        )

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
                    f'{yaw_err:.6f}'  if got_dock_goal else '',
                    f'{gx:.4f}', f'{gy:.4f}',
                    f'{math.degrees(gyaw):.2f}',
                ])

        self.latest_vx = 0.0
        self.latest_wz = 0.0

    # ── CSV event logger ──────────────────────────────────────────────────────

    def _init_csv(self):
        os.makedirs(self._CSV_DIR, exist_ok=True)
        with self._csv_lock:
            with open(self._CSV_PATH, 'w', newline='') as f:
                csv.writer(f).writerow(['timestamp_iso', 'event_type', 'description', 'value'])

    def log_event(self, event_type: str, description: str, value: str = ''):
        now = datetime.now().strftime('%Y-%m-%dT%H:%M:%S.%f')
        with self._csv_lock:
            with open(self._CSV_PATH, 'a', newline='') as f:
                csv.writer(f).writerow([now, event_type, description, value])

    # ── servo debug CSV ───────────────────────────────────────────────────────

    def _init_servo_debug_csv(self):
        attempt_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._servo_debug_CSV = os.path.join(self._CSV_DIR, f'servo_debug_{attempt_id}.csv')
        self._run_start_time  = time.time()
        os.makedirs(self._CSV_DIR, exist_ok=True)
        self.get_logger().info(f'[SERVO] New debug CSV: servo_debug_{attempt_id}.csv')
        with self._csv_lock:
            with open(self._servo_debug_CSV, 'w', newline='') as f:
                csv.writer(f).writerow([
                    # bookkeeping
                    'wall_time', 'elapsed_s', 'phase',
                    # camera frame (raw sensor)
                    'cam_tag_x_m', 'cam_tag_z_m', 'cam_tag_dist_m',
                    'cam_marker_yaw_deg',
                    # odom frame — robot
                    'robot_x', 'robot_y', 'robot_yaw_deg',
                    # odom frame — tag & dock
                    'tag_odom_x', 'tag_odom_y', 'dock_approach_yaw_deg',
                    # pure-pursuit geometry
                    't_robot_m', 'lat_err_m',
                    't_carrot_m', 'carrot_odom_x', 'carrot_odom_y',
                    'angle_to_carrot_deg', 'heading_err_deg',
                    # controller pipeline (separate stages so we can see where saturation hits)
                    'wz_proportional',   # k_steer * heading_err (raw P output)
                    'wz_after_deadband', # after friction boost / zero-out
                    'wz_commanded',      # after max_wz clamp  ← what goes to robot
                    'vx_commanded',
                    # active parameter snapshot (can change mid-run via ros2 param set)
                    'k_steer', 'friction_deadband', 'lookahead_dist',
                    'max_wz', 'v_linear',
                    # what the robot actually published (from /cmd_vel echo)
                    'actual_vx', 'actual_wz',
                ])

    def _log_servo_tick(self, phase: str,
                        cam_tag_x: float, cam_tag_z: float,
                        rx: float, ry: float, ryaw: float,
                        tag_ox: float, tag_oy: float, dock_yaw: float,
                        t_robot: float, lat_err: float,
                        t_carrot: float, carrot_ox: float, carrot_oy: float,
                        angle_to_carrot: float, heading_error: float,
                        wz_prop: float, wz_deadband: float, wz_cmd: float,
                        vx_cmd: float):
        if not self._servo_debug_CSV:
            return
        elapsed = time.time() - self._run_start_time
        cam_dist = math.sqrt(cam_tag_x ** 2 + cam_tag_z ** 2)
        with self._csv_lock:
            with open(self._servo_debug_CSV, 'a', newline='') as f:
                csv.writer(f).writerow([
                    f'{time.time():.4f}', f'{elapsed:.3f}', phase,
                    f'{cam_tag_x:.4f}', f'{cam_tag_z:.4f}', f'{cam_dist:.4f}',
                    f'{math.degrees(self._last_servo_marker_yaw):.2f}',
                    f'{rx:.4f}', f'{ry:.4f}', f'{math.degrees(ryaw):.2f}',
                    f'{tag_ox:.4f}', f'{tag_oy:.4f}', f'{math.degrees(dock_yaw):.2f}',
                    f'{t_robot:.4f}', f'{lat_err:.4f}',
                    f'{t_carrot:.4f}', f'{carrot_ox:.4f}', f'{carrot_oy:.4f}',
                    f'{math.degrees(angle_to_carrot):.2f}',
                    f'{math.degrees(heading_error):.2f}',
                    f'{wz_prop:.4f}', f'{wz_deadband:.4f}', f'{wz_cmd:.4f}',
                    f'{vx_cmd:.4f}',
                    f'{self.k_steer:.3f}', f'{self.friction_deadband:.3f}',
                    f'{self.lookahead_dist:.3f}', f'{self.max_wz:.3f}',
                    f'{self.v_linear:.3f}',
                    f'{self.latest_vx:.4f}', f'{self.latest_wz:.4f}',
                ])

    # ── VISUAL SERVO CONTROL LOOP (10 Hz) ────────────────────────────────────
    # Phase 1: Pure Pursuit (virtual carrot) until BLIND_APPROACH_THRESHOLD.
    # Phase 2: Blind dead-reckoning (encoder odometry only) for the final
    #          centimetres where the camera loses accuracy near the tag.

    def _servo_control_loop(self):
        with self._lock:
            if self._state != DockState.VISUAL_SERVO_DOCK:
                return

        # ── Phase 2: Blind dead-reckoning ────────────────────────────────────
        if self.in_blind_approach:
            rx, ry, _ = self._last_odom_pose
            sx, sy    = self.blind_start_pose
            dist_traveled = math.sqrt((rx - sx) ** 2 + (ry - sy) ** 2)

            if dist_traveled >= self.blind_target_dist:
                self._stop_robot()
                self.get_logger().info(
                    f'Blind approach complete — standoff {self.target_standoff:.2f} m reached. '
                    'MISSION SUCCESS'
                )
                self.log_event('MISSION_SUCCESS', 'Blind approach: standoff reached',
                               f'traveled={dist_traveled:.3f} target={self.blind_target_dist:.3f}')
                with self._lock:
                    self._state = DockState.DOCKED
                self._log_mission_end('SUCCESS')
                return

            twist = Twist()
            twist.linear.x  = self.v_linear
            twist.angular.z = 0.0
            rx, ry, ryaw = self._last_odom_pose
            tag_ox, tag_oy = self._last_servo_tag_odom
            _, _, dock_yaw = self._reference_dock_pose
            self._log_servo_tick(
                phase='BLIND',
                cam_tag_x=self._last_servo_marker_x, cam_tag_z=self._last_servo_marker_z,
                rx=rx, ry=ry, ryaw=ryaw,
                tag_ox=tag_ox, tag_oy=tag_oy, dock_yaw=dock_yaw,
                t_robot=0.0, lat_err=0.0,
                t_carrot=0.0, carrot_ox=0.0, carrot_oy=0.0,
                angle_to_carrot=0.0, heading_error=0.0,
                wz_prop=0.0, wz_deadband=0.0, wz_cmd=0.0,
                vx_cmd=self.v_linear,
            )
            self._cmd_pub.publish(twist)
            return

        # ── Safety: tag-lost handling ─────────────────────────────────────────
        tag_age = time.time() - self._last_servo_detection_time
        if tag_age > 0.5 and self._last_servo_marker_z < 0.5:
            # Tag went out of view at close range — estimate remaining distance
            # from last known z, compensating for travel since detection was lost.
            est_z   = max(0.0, self._last_servo_marker_z - tag_age * self.v_linear)
            remaining = est_z - self.target_standoff
            rx, ry, _ = self._last_odom_pose
            self.get_logger().warn(
                f'Tag lost {tag_age:.2f}s ago (last_z={self._last_servo_marker_z:.3f}m, '
                f'est_z={est_z:.3f}m) — handoff to blind approach, remaining={remaining:.3f}m'
            )
            self.log_event('BLIND_APPROACH', 'Tag lost at close range — blind handoff',
                           f'last_z={self._last_servo_marker_z:.3f} est_z={est_z:.3f} '
                           f'tag_age={tag_age:.2f}')
            if remaining <= 0.0:
                self._stop_robot()
                self.log_event('MISSION_SUCCESS', 'Already at standoff on tag-lost handoff', '')
                with self._lock:
                    self._state = DockState.DOCKED
                self._log_mission_end('SUCCESS')
                return
            self.blind_start_pose  = (rx, ry)
            self.blind_target_dist = remaining
            self.in_blind_approach = True
            twist = Twist()
            twist.linear.x  = self.v_linear
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            return
        if tag_age > 2.0:
            self.get_logger().error('Visual servo: marker-3 lost for >2 s — ABORTED')
            self.log_event('SAFETY_ABORT', 'Visual servo: marker lost >2 s')
            self._stop_robot()
            with self._lock:
                self._state = DockState.ABORTED
            self._log_mission_end('ABORT')
            return

        marker_z   = self._last_servo_marker_z
        marker_x   = self._last_servo_marker_x
        marker_yaw = self._last_servo_marker_yaw

        # ── Handoff: enter blind approach when close enough ───────────────────
        if marker_z <= self.blind_approach_threshold:
            remaining = self.blind_approach_threshold - self.target_standoff
            if remaining <= 0.0:
                self._stop_robot()
                self.get_logger().info('Already within standoff — MISSION SUCCESS')
                self.log_event('MISSION_SUCCESS', 'Already at standoff on blind handoff',
                               f'marker_z={marker_z:.3f}')
                with self._lock:
                    self._state = DockState.DOCKED
                self._log_mission_end('SUCCESS')
                return
            rx, ry, _          = self._last_odom_pose
            self.blind_start_pose  = (rx, ry)
            self.blind_target_dist = remaining
            self.in_blind_approach = True
            self.get_logger().info(
                f'Entering blind approach — marker_z={marker_z:.3f} m, '
                f'driving {remaining:.3f} m straight by encoders'
            )
            self.log_event('BLIND_APPROACH', 'Handoff to dead-reckoning',
                           f'marker_z={marker_z:.3f} target={remaining:.3f}')
            twist = Twist()
            twist.linear.x  = self.v_linear
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            return

        # ── Camera-frame pure-pursuit servo ──────────────────────────────────
        # All geometry in camera_optical XZ plane (X = right, Z = forward).
        # Robot/camera sits at the origin (0, 0).
        #
        # d   = (Xt, Zt)  — vector from camera to tag
        # Z3t = (z3t_hat_x, z3t_hat_z) — tag Z-axis in camera XZ (points INTO dock)
        #
        # ThetaT: signed angle from Z3t to d.
        #   Negative → tag is left of camera centre → robot is RIGHT of centreline.
        #   Positive → tag is right of camera centre → robot is LEFT  of centreline.
        #
        # lat_err = |d| · sin(ThetaT)
        #   This is the perpendicular distance from the origin to the Z3t line.
        #   (sin gives the perpendicular component; cos gives the along-track component.)
        #
        # Carrot = foot-of-perpendicular from origin to Z3t line + lookahead · Z3t_hat
        #   The foot is the closest point on the centreline to the robot.
        #   Adding lookahead along Z3t places the carrot ahead of that foot toward dock.
        #
        # heading_err: bearing from camera +Z to carrot, CCW positive.
        #   Positive → carrot is to the LEFT → turn left (positive wz). ✓
        #
        # vx is reduced by cos(heading_err) so the robot slows when heavily off-angle,
        # giving the steering more time to converge before closing distance.

        # snapshot (guard against concurrent _on_any_marker update)
        Xt = self._last_servo_marker_x    # cam_x  (right = positive)
        Zt = self._last_servo_marker_z    # cam_z  (forward = positive)
        z_base_ax, z_base_ay = self._last_servo_z_base
        # recover camera-frame components stored in base_link convention:
        #   _last_servo_z_base = (z_cam_z, -z_cam_x)
        z3t_raw_x = -z_base_ay           # z_cam_x = -z_base_y
        z3t_raw_z =  z_base_ax           # z_cam_z =  z_base_x

        d_mag = math.hypot(Xt, Zt)

        # normalise Z3t; flip if it points away from dock (opposite to d)
        z3t_norm = math.hypot(z3t_raw_x, z3t_raw_z)
        if z3t_norm < 1e-6:
            z3t_norm = 1.0
        z3t_hat_x = z3t_raw_x / z3t_norm
        z3t_hat_z = z3t_raw_z / z3t_norm
        if z3t_hat_x * Xt + z3t_hat_z * Zt < 0.0:
            z3t_hat_x = -z3t_hat_x
            z3t_hat_z = -z3t_hat_z

        # ThetaT: signed angle from Z3t to d
        #   cross = Xt·z3t_z − Zt·z3t_x  (positive when d is CCW / left of Z3t)
        #   dot   = Xt·z3t_x + Zt·z3t_z
        theta_t = math.atan2(
            Xt * z3t_hat_z - Zt * z3t_hat_x,
            Xt * z3t_hat_x + Zt * z3t_hat_z,
        )

        # lateral error: perpendicular distance from origin to Z3t centreline
        lat_err = d_mag * math.sin(theta_t)   # + = robot left, − = robot right

        # foot-of-perpendicular from origin onto Z3t line
        t_proj    = -(Xt * z3t_hat_x + Zt * z3t_hat_z)
        foot_cam_x = Xt + t_proj * z3t_hat_x
        foot_cam_z = Zt + t_proj * z3t_hat_z

        # carrot: foot + lookahead distance along Z3t toward dock
        carrot_cam_x = foot_cam_x + self.lookahead_dist * z3t_hat_x
        carrot_cam_z = foot_cam_z + self.lookahead_dist * z3t_hat_z
        if carrot_cam_z < 0.02:          # safety: keep carrot in front of robot
            carrot_cam_z = 0.02

        L_carrot = math.hypot(carrot_cam_x, carrot_cam_z)

        # heading error: CCW angle from camera +Z to carrot (positive = turn left)
        heading_err = math.atan2(-carrot_cam_x, carrot_cam_z)

        # reduce forward speed when heavily off-angle; never below 0.05 m/s
        heading_factor = max(0.0, math.cos(heading_err))
        vx_cmd = max(0.05, self.v_linear * heading_factor)

        wz_prop = (self.k_steer * vx_cmd * 2.0 * math.sin(heading_err) / L_carrot
                   if L_carrot > 0.02 else 0.0)

        wz_deadband = wz_prop
        if abs(heading_err) > 0.02:
            if 0.0 < wz_deadband < self.friction_deadband:
                wz_deadband = self.friction_deadband
            elif -self.friction_deadband < wz_deadband < 0.0:
                wz_deadband = -self.friction_deadband
        else:
            wz_deadband = 0.0

        wz_cmd = max(-self.max_wz, min(self.max_wz, wz_deadband))

        twist = Twist()
        twist.linear.x  = vx_cmd
        twist.angular.z = wz_cmd

        t_carrot_along = t_proj + self.lookahead_dist
        self.get_logger().info(
            f'[SERVO] '
            f'd=({Xt:.3f},{Zt:.3f})m '
            f'Z3t=({z3t_hat_x:.3f},{z3t_hat_z:.3f}) '
            f'θT={math.degrees(theta_t):.1f}° '
            f'lat={lat_err:.3f}m '
            f't_robot={t_proj:.3f}m '
            f't_carrot={t_carrot_along:.3f}m '
            f'hdg={math.degrees(heading_err):.1f}° '
            f'carrot=({carrot_cam_x:.3f},{carrot_cam_z:.3f})m '
            f'wz={wz_cmd:.2f}'
        )

        rx, ry, ryaw = self._last_odom_pose
        self._log_servo_tick(
            phase='SERVO',
            cam_tag_x=Xt, cam_tag_z=Zt,
            rx=rx, ry=ry, ryaw=ryaw,
            tag_ox=z3t_hat_x, tag_oy=z3t_hat_z,            # Z3t unit vector (repurposed)
            dock_yaw=theta_t,                                # ThetaT in rad  (repurposed)
            t_robot=t_proj,                                  # foot along-track from tag
            lat_err=lat_err,
            t_carrot=t_proj + self.lookahead_dist,
            carrot_ox=carrot_cam_x, carrot_oy=carrot_cam_z, # camera frame   (repurposed)
            angle_to_carrot=math.atan2(carrot_cam_x, carrot_cam_z),
            heading_error=heading_err,
            wz_prop=wz_prop, wz_deadband=wz_deadband, wz_cmd=wz_cmd,
            vx_cmd=vx_cmd,
        )
        self._cmd_pub.publish(twist)

    # ── spiral search ─────────────────────────────────────────────────────────

    def _fire_local_search(self):
        x0, y0, _ = self._last_odom_pose
        waypoints  = _gen_spiral_waypoints(self, x0, y0)

        self.get_logger().info(
            f'Spiral: origin=({x0:.2f},{y0:.2f})  {len(waypoints)} waypoints'
        )
        self.log_event('STATE_CHANGE', 'Transitioned to LOCAL_SEARCH',
                       f'origin=({x0:.2f},{y0:.2f})')

        if not self._spiral_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateThroughPoses unavailable — ABORTED')
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
            self.get_logger().error('Spiral goal rejected — IDLE')
            self.log_event('ERROR', 'NavigateThroughPoses goal rejected')
            with self._lock:
                self._state = DockState.IDLE
            return
        self.log_event('GOAL_ACCEPTED', 'Spiral accepted')
        with self._lock:
            self._current_goal_handle = handle
        handle.get_result_async().add_done_callback(self._local_search_done_cb)

    def _local_search_done_cb(self, future):
        result = future.result()
        status = result.status if result else GoalStatus.STATUS_ABORTED

        with self._lock:
            state = self._state

        if state in (DockState.BUFFERING_ROUGH, DockState.VISUAL_HOMING,
                     DockState.VISUAL_SERVO_DOCK, DockState.DOCKED):
            return  # tag found mid-spiral — docking path already active

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error('Spiral complete — target not found. ABORTED.')
            self.log_event('SAFETY_ABORT', 'Target not found after spiral')
            with self._lock:
                self._state = DockState.ABORTED
            self._stop_robot()
        else:
            self.get_logger().warn(f'Spiral ended unexpectedly (status={status}) — IDLE')
            with self._lock:
                self._state = DockState.IDLE

    def _zone_nav_done_cb(self, future):
        result = future.result()
        status = result.status if result else GoalStatus.STATUS_ABORTED

        with self._lock:
            state = self._state

        if state != DockState.COMMUTING:
            return  # tag detected en route — docking path already active

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Zone reached — no tag seen, starting LOCAL_SEARCH spiral')
            self.log_event('STATE_CHANGE', 'Transitioned to LOCAL_SEARCH',
                           'zone reached with no tag detection')
            with self._lock:
                self._state = DockState.LOCAL_SEARCH
            self._fire_local_search()
        else:
            self.get_logger().info(f'Zone nav ended (status={status}) in COMMUTING — awaiting goal')

    # ── box_center_pose — fills BUFFERING_ROUGH only ──────────────────────────

    def _on_box_pose(self, msg: PoseStamped):
        """Accumulates box_center_pose samples. All state transitions are driven
        by _on_any_marker (camera-frame distances); box_pose is only consumed
        once the robot is already close and collecting accurate odom-frame data."""
        q = msg.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        sample = (msg.pose.position.x, msg.pose.position.y, yaw)

        fire_staging = False

        with self._lock:
            if self._state in (DockState.IDLE, DockState.DOCKED, DockState.ABORTED,
                               DockState.COMMUTING, DockState.LOCAL_SEARCH,
                               DockState.VISUAL_HOMING, DockState.VISUAL_SERVO_DOCK):
                return

            if self._state == DockState.BUFFERING_ROUGH:
                self._rough_buf.append(sample)
                if len(self._rough_buf) >= self.BUF_ROUGH:
                    self._state  = DockState.NAV_TO_STAGING
                    fire_staging = True

        if fire_staging:
            with self._lock:
                buf = list(self._rough_buf)
            self._fire_staging_goal(buf)

    # ── marker callbacks ──────────────────────────────────────────────────────

    def _marker_cam_to_odom(self, p) -> tuple[float, float]:
        """Transform camera_optical position to odom (x, y).

        Hardcoded static camera_optical→base_link transform from launch file:
          translation x=0.126 m, yaw=-π/2, roll=-π/2
          rotation matrix: [[0,0,1], [-1,0,0], [0,-1,0]]
          bx = p.z + 0.126   (camera Z → robot forward + lens offset)
          by = -p.x           (camera X right → robot Y left, negated)
        Then rotate by robot yaw to get odom frame.
        """
        bx = p.z + 0.126
        by = -p.x
        rx, ry, ryaw = self._last_odom_pose
        ox = rx + bx * math.cos(ryaw) - by * math.sin(ryaw)
        oy = ry + bx * math.sin(ryaw) + by * math.cos(ryaw)
        return ox, oy

    def _on_any_marker(self, msg: PoseStamped, marker_id: int):
        """All state-machine transitions driven by camera-frame marker distance.

        Priority:
          1. Marker-3 precision → VISUAL_SERVO_DOCK   (dist < PROXIMITY_TRIGGER_M)
          2. Homing entry       → VISUAL_HOMING        (any marker, dist > VISUAL_HOMING_CLOSE_M)
          3. Direct buffering   → BUFFERING_ROUGH       (any marker, dist ≤ VISUAL_HOMING_CLOSE_M)
          4. Homing exit        → BUFFERING_ROUGH       (during VISUAL_HOMING, dist ≤ VISUAL_HOMING_CLOSE_M)
        """
        p    = msg.pose.position
        dist = math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2)

        self._marker_seen[marker_id] = (self.get_clock().now().nanoseconds, dist)

        # Always update visual servo data for marker 3 (needed in servo loop)
        if marker_id == 3:
            self._last_servo_marker_x = p.x
            self._last_servo_marker_z = p.z
            q = msg.pose.orientation
            qx, qy, qz, qw = q.x, q.y, q.z, q.w

            # Compute the dock's true approach direction from the tag's Z-axis.
            # Tag Z in camera_optical (rotate [0,0,1] by quaternion):
            z_cam_x = 2.0 * (qx * qz + qy * qw)
            z_cam_z = 1.0 - 2.0 * (qx * qx + qy * qy)
            # Heading error: signed angle from camera +Z to tag normal in XZ plane.
            self._last_servo_marker_yaw = math.atan2(z_cam_x, z_cam_z)
            # Transform to base_link (cam_z→base_x, -cam_x→base_y):
            z_base_x =  z_cam_z
            z_base_y = -z_cam_x
            self._last_servo_z_base = (z_base_x, z_base_y)  # approach dir in base_link
            # Rotate to odom frame using current robot yaw:
            _, _, ryaw = self._last_odom_pose
            self._last_servo_dock_yaw = self._last_servo_marker_yaw
            self._last_servo_detection_time = time.time()
            self._last_servo_tag_odom = self._marker_cam_to_odom(p)
        action:    str   = ''
        homing_ox: float = 0.0
        homing_oy: float = 0.0

        with self._lock:
            # ── 1. Marker-3 precision → VISUAL_SERVO_DOCK ─────────────────
            if (marker_id == 3
                    and self._state in (DockState.COMMUTING, DockState.LOCAL_SEARCH,
                                        DockState.VISUAL_HOMING, DockState.NAV_TO_STAGING)
                    and dist < self.PROXIMITY_TRIGGER_M):
                self._state = DockState.VISUAL_SERVO_DOCK
                action = 'servo_dock'

            # ── 2 & 3. Homing entry / direct buffering ─────────────────────
            elif self._state in (DockState.COMMUTING, DockState.LOCAL_SEARCH):
                if dist > self.VISUAL_HOMING_CLOSE_M:
                    self._state = DockState.VISUAL_HOMING
                    action = 'homing_enter'
                    homing_ox, homing_oy = self._marker_cam_to_odom(p)
                else:
                    self._rough_buf.clear()
                    self._state = DockState.BUFFERING_ROUGH
                    action = 'buffering_direct'

            # ── 4. Homing exit ─────────────────────────────────────────────
            elif self._state == DockState.VISUAL_HOMING:
                if dist <= self.VISUAL_HOMING_CLOSE_M:
                    self._rough_buf.clear()
                    self._state = DockState.BUFFERING_ROUGH
                    action = 'homing_exit'

        # ── execute outside lock ──────────────────────────────────────────
        if action == 'servo_dock':
            self.get_logger().info(
                f'Marker-3 at {dist:.2f} m — cancelling RPP, VISUAL_SERVO_DOCK'
            )
            self.log_event('STATE_CHANGE', 'Transitioned to VISUAL_SERVO_DOCK',
                           f'marker3_dist={dist:.3f}')
            self.in_blind_approach       = False
            self.blind_start_pose        = None
            self._servo_approach_yaw     = None   # will be frozen on first servo tick
            self._servo_tag_odom_frozen  = None   # will be frozen on first servo tick
            self._init_servo_debug_csv()          # fresh file for this docking attempt
            self._cancel_current_goal()           # hand cmd_vel exclusively to servo loop

        elif action == 'homing_enter':
            self.get_logger().info(
                f'Marker {marker_id} at {dist:.2f} m — VISUAL_HOMING '
                f'(odom ≈ {homing_ox:.2f},{homing_oy:.2f})'
            )
            self.log_event('STATE_CHANGE', 'Transitioned to VISUAL_HOMING',
                           f'marker{marker_id}_dist={dist:.3f}')
            self._cancel_current_goal()
            self._fire_visual_homing_goal(homing_ox, homing_oy)

        elif action == 'buffering_direct':
            self.get_logger().info(
                f'Marker {marker_id} at {dist:.2f} m — BUFFERING_ROUGH (direct)'
            )
            self.log_event('STATE_CHANGE', 'Transitioned to BUFFERING_ROUGH',
                           f'marker{marker_id}_dist={dist:.3f} (direct)')
            self._cancel_current_goal()

        elif action == 'homing_exit':
            self.get_logger().info(
                f'Visual homing: marker {marker_id} now at {dist:.2f} m — BUFFERING_ROUGH'
            )
            self.log_event('STATE_CHANGE', 'Transitioned to BUFFERING_ROUGH',
                           f'marker{marker_id}_dist={dist:.3f} (homing exit)')
            self._cancel_current_goal()

    # ── mission reset helper ─────────────────────────────────────────────────

    def _reset_mission(self):
        """Clean reset of all mission state. Safe to call from any state."""
        self._cancel_current_goal()
        self._stop_robot()

        with self._lock:
            prev_state = self._state

        if prev_state not in (DockState.IDLE, DockState.DOCKED, DockState.ABORTED):
            self._log_mission_end('CANCELLED')

        self.in_blind_approach          = False
        self.blind_start_pose           = None
        self.blind_target_dist          = 0.0
        self._last_servo_detection_time = time.time()
        self._servo_approach_yaw        = None
        self._servo_tag_odom_frozen     = None
        self._reference_dock_pose       = (0.0, 0.0, 0.0)

        with self._lock:
            self._state               = DockState.IDLE
            self._rough_buf.clear()
            self._current_goal_handle = None
            self._mission_logged      = False

        self.log_event('RESET', 'Mission reset — returned to IDLE')
        self.get_logger().info(
            'Mission reset triggered. State returned to IDLE. Variables cleared.'
        )

    # ── dynamic parameter callback ────────────────────────────────────────────

    def _parameter_callback(self, params):
        for p in params:
            if   p.name == 'lookahead_dist':           self.lookahead_dist           = p.value
            elif p.name == 'k_steer':                  self.k_steer                  = p.value
            elif p.name == 'friction_deadband':        self.friction_deadband        = p.value
            elif p.name == 'max_wz':                   self.max_wz                   = p.value
            elif p.name == 'v_linear':                 self.v_linear                 = p.value
            elif p.name == 'target_standoff':          self.target_standoff          = p.value
            elif p.name == 'blind_approach_threshold': self.blind_approach_threshold = p.value
            else:
                continue
            self.get_logger().info(f'[PARAM] {p.name} → {p.value}')
        return SetParametersResult(successful=True)

    # ── RViz goal proxy ───────────────────────────────────────────────────────

    def _on_goal_pose(self, msg: PoseStamped):
        with self._lock:
            state = self._state

        # Auto-reset from terminal states so new goals are never ignored
        if state in (DockState.DOCKED, DockState.ABORTED):
            self.get_logger().info(
                f'New goal received in {state.name} — auto-resetting mission'
            )
            self._reset_mission()  # sets state → IDLE, _mission_logged → False

        with self._lock:
            state = self._state
            if state == DockState.IDLE:
                self._state = DockState.COMMUTING
                self._rough_buf.clear()
                do_cancel = False
            elif state == DockState.COMMUTING:
                self._rough_buf.clear()
                do_cancel = True
            else:
                self.get_logger().info(f'/goal_pose ignored — state is {state.name}')
                return

        if do_cancel:
            self._cancel_current_goal()
            self.get_logger().info(
                f'New /goal_pose — re-targeting '
                f'({msg.pose.position.x:.2f},{msg.pose.position.y:.2f})'
            )
        else:
            self._mission_logged = False
            self._run_start_time = time.time()
            self.get_logger().info(
                f'IDLE → COMMUTING — zone '
                f'({msg.pose.position.x:.2f},{msg.pose.position.y:.2f})'
            )

        self.log_event('GOAL_POSE', 'RViz zone goal received',
                       f'({msg.pose.position.x:.2f},{msg.pose.position.y:.2f})')
        msg.header.stamp = rclpy.time.Time().to_msg()
        self._send_nav_goal(msg, self._zone_nav_done_cb)

    # ── odom ──────────────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        q  = msg.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._last_odom_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )

    # ── visual homing ─────────────────────────────────────────────────────────

    def _fire_visual_homing_goal(self, ox: float, oy: float):
        """Send RPP nav goal toward the marker's odom position.

        Standoff is inside the VISUAL_HOMING_CLOSE_M camera trigger, so
        _on_any_marker will cancel this goal and enter BUFFERING_ROUGH before
        the robot physically arrives at the nav standoff.
        """
        rx, ry, _ = self._last_odom_pose
        approach_angle = math.atan2(oy - ry, ox - rx)

        hx   = ox - self.VISUAL_HOMING_NAV_STANDOFF_M * math.cos(approach_angle)
        hy   = oy - self.VISUAL_HOMING_NAV_STANDOFF_M * math.sin(approach_angle)
        hyaw = approach_angle

        goal_pose = _pose_stamped(hx, hy, hyaw, 'odom')
        self.get_logger().info(
            f'Visual homing nav → ({hx:.3f},{hy:.3f}) yaw={math.degrees(hyaw):.1f}°  '
            f'marker_odom=({ox:.3f},{oy:.3f})'
        )
        self.log_event('GOAL_SENT', 'Visual homing nav goal sent',
                       f'standoff=({hx:.3f},{hy:.3f})')
        self._send_nav_goal(goal_pose, self._visual_homing_done_cb)

    def _visual_homing_done_cb(self, future):
        result = future.result()
        status = result.status if result else GoalStatus.STATUS_ABORTED

        with self._lock:
            state = self._state

        if state != DockState.VISUAL_HOMING:
            return  # already transitioned by _on_any_marker — nothing to do

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Visual homing goal reached — BUFFERING_ROUGH')
            self.log_event('STATE_CHANGE', 'Transitioned to BUFFERING_ROUGH',
                           'visual homing nav succeeded')
            with self._lock:
                self._state = DockState.BUFFERING_ROUGH
                self._rough_buf.clear()
        else:
            self.get_logger().warn(
                f'Visual homing failed (status={status}) — falling back to LOCAL_SEARCH'
            )
            with self._lock:
                self._state = DockState.LOCAL_SEARCH
            self._fire_local_search()

    # ── staging ───────────────────────────────────────────────────────────────

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
            self._reference_dock_pose = (avg_x, avg_y, avg_yaw)

        goal_pose = _pose_stamped(sx, sy, syaw, 'odom')
        self.get_logger().info(
            f'Staging → ({sx:.3f},{sy:.3f}) yaw={math.degrees(syaw):.1f}°'
        )
        self.log_event('STATE_CHANGE', 'Transitioned to NAV_TO_STAGING',
                       f'goal=({sx:.3f},{sy:.3f}) yaw={math.degrees(syaw):.1f}deg')
        self._send_nav_goal(goal_pose, self._staging_done_cb)

    def _staging_done_cb(self, future):
        result = future.result()
        status = result.status if result else GoalStatus.STATUS_ABORTED

        with self._lock:
            state = self._state

        if state == DockState.VISUAL_SERVO_DOCK:
            return  # precision trigger fired en route — already in servo mode

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Staging failed (status={status}) — IDLE')
            self.log_event('STATE_CHANGE', 'Transitioned to IDLE',
                           f'staging_failed status={status}')
            with self._lock:
                self._state = DockState.IDLE
        else:
            self.get_logger().info('Staging reached — entering visual servo')
            self.log_event('STATE_CHANGE', 'Transitioned to VISUAL_SERVO_DOCK',
                           'staging_done_cb handoff')
            self.in_blind_approach = False
            self.blind_start_pose  = None
            with self._lock:
                self._state = DockState.VISUAL_SERVO_DOCK

    # ── Nav2 action helpers ───────────────────────────────────────────────────

    def _send_nav_goal(self, pose: PoseStamped, done_cb):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available — IDLE')
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
            self.get_logger().error('Nav2 rejected goal — IDLE')
            self.log_event('ERROR', 'Nav2 goal rejected')
            with self._lock:
                self._state = DockState.IDLE
            return
        self.log_event('GOAL_ACCEPTED', 'Nav2 goal accepted')
        with self._lock:
            self._current_goal_handle = handle
        handle.get_result_async().add_done_callback(done_cb)

    def _cancel_current_goal(self):
        with self._lock:
            handle = self._current_goal_handle
        if handle is not None:
            handle.cancel_goal_async()

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

        if pre_reset_state == DockState.DOCKED:
            self._log_mission_end('SUCCESS')
        elif pre_reset_state == DockState.ABORTED:
            self._log_mission_end('ABORT')
        elif pre_reset_state not in (DockState.IDLE,):
            self._log_mission_end('CANCELLED')

        self.in_blind_approach = False
        self.blind_start_pose  = None
        self.blind_target_dist = 0.0
        with self._lock:
            self._state = DockState.IDLE
            self._rough_buf.clear()
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
