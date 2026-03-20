#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.msg import SpeedLimit
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations as tft
import numpy as np
import math
from collections import deque
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Polygon, Point32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import threading


class BoxEstimator(Node):
    def __init__(self):
        super().__init__('box_estimator')

        # --- CONFIGURATION ---
        self.BOX_LENGTH = 0.204
        self.BOX_WIDTH  = 0.20
        self.half_L = self.BOX_LENGTH / 2.0
        self.half_W = self.BOX_WIDTH / 2.0

        # Rough approach stops 0.5m in front of box
        # Precision approach stops 0.15m in front of box
        self.APPROACH_DIST_ROUGH     = 0.5
        self.APPROACH_DIST_PRECISION = 0.15

        self.target_ids = [0, 1, 2, 3]
        self.footprint_shrunk = False

        # --- GEOMETRY ---
        self.marker_transforms = {}
        self.setup_box_geometry()

        # ================================================================
        # STATE MACHINE — exactly two goals are ever sent:
        #
        #   PHASE 1 — ROUGH APPROACH (any marker)
        #     Collect ROUGH_BUFFER_SIZE frames → strip outliers → stable pose
        #     Send ONE goal to Nav2 via NavigateToPose action
        #     Robot drives to APPROACH_DIST_ROUGH in front of box
        #     No further goal updates until precision mode triggers
        #
        #   PHASE 2 — PRECISION (marker 3 only, dist ≤ PRECISION_TRIGGER_DIST)
        #     Collect PRECISION_BUFFER_SIZE frames → strip outliers → pristine pose
        #     Send ONE final goal via new NavigateToPose action
        #     Robot drives slowly to APPROACH_DIST_PRECISION and stops
        #     docking_locked = True — nothing ever sent again
        # ================================================================

        # Phase 1
        self.ROUGH_BUFFER_SIZE  = 20
        self.rough_buffer       = []
        self.rough_goal_sent    = False
        self.navigation_started = False

        # Phase 2
        self.PRECISION_TRIGGER_DIST = 1.0
        self.PRECISION_BUFFER_SIZE  = 20
        self.precision_mode         = False
        self.precision_buffer       = []
        self.docking_locked         = False

        # Shared
        self.last_detection_time = {}
        self.last_robot_x        = 0.0
        self.last_robot_y        = 0.0
        self.last_log_time       = {}

        # --- ROS SETUP ---
        self.cb_group    = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.update_pub           = self.create_publisher(PoseStamped, '/goal_update', 10)
        self.mask_pub             = self.create_publisher(PoseStamped, '/parking_goal', 10)
        self.box_pub              = self.create_publisher(PoseStamped, '/box_center_pose', 10)
        self.viz_pub              = self.create_publisher(MarkerArray, '/box_debug_axes', 10)
        self.speed_limit_pub      = self.create_publisher(SpeedLimit,  '/speed_limit', 10)
        self.local_footprint_pub  = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.global_footprint_pub = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.rviz_goal_pub        = self.create_publisher(PoseStamped, '/visual_target_goal', 10)

        # Extended TF buffer — 30s prevents SLAM hiccups from causing
        # lookup failures that force fallback to latest transform
        self.tf_buffer   = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS depth=1 BEST_EFFORT — drops stale frames immediately
        marker_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subs = []
        for m_id in self.target_ids:
            sub = self.create_subscription(
                PoseStamped,
                f'/aruco/marker_{m_id}',
                lambda msg, mid=m_id: self.detection_callback(msg, mid),
                marker_qos,
                callback_group=self.cb_group
            )
            self.subs.append(sub)

        self.mission_active = False
        self.start_sub = self.create_subscription(
            Bool, '/start_mission',
            self.start_callback, 10,
            callback_group=self.cb_group
        )

        self.get_logger().info(
            f"Box Estimator Ready | "
            f"Phase 1: {self.ROUGH_BUFFER_SIZE} frames → 1 rough goal @ {self.APPROACH_DIST_ROUGH}m | "
            f"Phase 2: {self.PRECISION_BUFFER_SIZE} frames → 1 final goal @ {self.APPROACH_DIST_PRECISION}m"
        )

    # ------------------------------------------------------------------
    # CALLBACKS
    # ------------------------------------------------------------------

    def start_callback(self, msg):
        if msg.data:
            self.get_logger().info("✅ MISSION ACTIVE")
            self.mission_active = True
        else:
            self.get_logger().info("🛑 MISSION DEACTIVATED")
            self.mission_active = False

    def detection_callback(self, msg, m_id):
        if not self.mission_active:
            return
        if self.docking_locked:
            return

        # Marker 3 priority — if marker 3 seen in last 500ms, discard all others
        # Prevents box centre jumping when side markers flicker
        now_ns    = self.get_clock().now().nanoseconds
        m3_age_ns = now_ns - self.last_detection_time.get(3, 0)
        if m_id != 3 and m3_age_ns < 500_000_000:
            return

        self.solve_box_pose(msg, m_id)

    # ------------------------------------------------------------------
    # GEOMETRY
    # ------------------------------------------------------------------

    def setup_box_geometry(self):
        def make_mat(pos, x_vec, y_vec, z_vec):
            mat = np.eye(4)
            mat[0:3, 0] = x_vec
            mat[0:3, 1] = y_vec
            mat[0:3, 2] = z_vec
            mat[0:3, 3] = pos
            return mat

        # Marker 1 — Left Face (+Y)
        self.marker_transforms[1] = make_mat(
            [0.0, self.half_W, 0.2], [-1, 0, 0], [0, 0, 1], [0, 1, 0])
        # Marker 0 — Right Face (-Y)
        self.marker_transforms[0] = make_mat(
            [0.0, -self.half_W, 0.2], [1, 0, 0], [0, 0, 1], [0, -1, 0])
        # Marker 2 — Back Face (-X)
        self.marker_transforms[2] = make_mat(
            [-self.half_L, 0.0, 0.2], [0, 1, 0], [0, 0, 1], [1, 0, 0])
        # Marker 3 — Front Face (+X)
        self.marker_transforms[3] = make_mat(
            [self.half_L - 0.06, 0.0, 0.2], [0, -1, 0], [0, 0, 1], [-1, 0, 0])

    # ------------------------------------------------------------------
    # CORE POSE ESTIMATION
    # ------------------------------------------------------------------

    def solve_box_pose(self, pose_msg, m_id):
        try:
            pose = pose_msg.pose

            # Camera → Marker transform
            t_cam_marker = tft.concatenate_matrices(
                tft.translation_matrix([pose.position.x, pose.position.y, pose.position.z]),
                tft.quaternion_matrix([pose.orientation.x, pose.orientation.y,
                                       pose.orientation.z, pose.orientation.w])
            )

            # Update last-seen timestamp immediately before any early returns
            with self._state_lock:
                self.last_detection_time[m_id] = self.get_clock().now().nanoseconds

            # Odom → Camera TF — use image capture timestamp for accuracy
            tf_stamped = None
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    'odom', pose_msg.header.frame_id,
                    pose_msg.header.stamp,
                    rclpy.duration.Duration(seconds=0.3)
                )
            except (LookupException, ConnectivityException, ExtrapolationException):
                try:
                    tf_stamped = self.tf_buffer.lookup_transform(
                        'odom', pose_msg.header.frame_id,
                        rclpy.time.Time(),
                        rclpy.duration.Duration(seconds=0.1)
                    )
                except (LookupException, ConnectivityException, ExtrapolationException) as e:
                    self.get_logger().warn(f"TF lookup failed: {e}",
                                           throttle_duration_sec=1.0)
                    return

            t = tf_stamped.transform.translation
            r = tf_stamped.transform.rotation
            t_map_cam = tft.concatenate_matrices(
                tft.translation_matrix([t.x, t.y, t.z]),
                tft.quaternion_matrix([r.x, r.y, r.z, r.w])
            )

            # Odom → Box
            t_box_marker = self.marker_transforms.get(m_id, np.eye(4))
            t_marker_box = tft.inverse_matrix(t_box_marker)
            t_map_box    = tft.concatenate_matrices(t_map_cam, t_cam_marker, t_marker_box)

            # Extract position BEFORE rot_180 — rotation only corrects yaw, not position
            curr_x = t_map_box[0, 3]
            curr_y = t_map_box[1, 3]

            rot_180        = tft.euler_matrix(0, 0, math.pi)
            t_map_box_yaw  = tft.concatenate_matrices(t_map_box, rot_180)
            _, _, curr_yaw = tft.euler_from_matrix(t_map_box_yaw)

            # Raw camera-to-marker distance — TF-independent, immune to SLAM jitter
            dist_cam_to_marker = math.sqrt(
                pose.position.x**2 + pose.position.y**2 + pose.position.z**2)

            # 1Hz debug log per marker
            now_ns = self.get_clock().now().nanoseconds
            if (now_ns - self.last_log_time.get(m_id, 0)) > 1e9:
                self.last_log_time[m_id] = now_ns
                self.get_logger().info(
                    f"[M{m_id}] dist={dist_cam_to_marker:.3f}m | "
                    f"box=({curr_x:.3f}, {curr_y:.3f}) | "
                    f"yaw={math.degrees(curr_yaw):.1f}° | "
                    f"rough_buf={len(self.rough_buffer)}/{self.ROUGH_BUFFER_SIZE} | "
                    f"prec_buf={len(self.precision_buffer)}/{self.PRECISION_BUFFER_SIZE} | "
                    f"rough_sent={self.rough_goal_sent} | locked={self.docking_locked}"
                )

            # ==============================================================
            # PHASE 2 — PRECISION MODE
            # Triggered once when marker 3 is within PRECISION_TRIGGER_DIST
            # Collects PRECISION_BUFFER_SIZE frames then sends ONE final goal
            # ==============================================================
            if m_id == 3 and dist_cam_to_marker <= self.PRECISION_TRIGGER_DIST \
                    and not self.precision_mode:
                self.precision_mode = True
                self.get_logger().info(
                    f"🎯 Precision Mode triggered! M3 dist={dist_cam_to_marker:.3f}m — "
                    f"collecting {self.PRECISION_BUFFER_SIZE} frames..."
                )
                # Slow the robot while collecting precision frames
                speed_msg             = SpeedLimit()
                speed_msg.percentage  = False
                speed_msg.speed_limit = 0.15
                self.speed_limit_pub.publish(speed_msg)

            if self.precision_mode:
                if m_id != 3:
                    return  # only marker 3 contributes to precision buffer

                with self._state_lock:
                    self.precision_buffer.append((curr_x, curr_y, curr_yaw))
                    buffer_len = len(self.precision_buffer)

                # Log progress at 1Hz
                if (now_ns - self.last_log_time.get('prec_buf', 0)) > 1e9:
                    self.last_log_time['prec_buf'] = now_ns
                    self.get_logger().info(
                        f"📦 Precision buffer: {buffer_len}/{self.PRECISION_BUFFER_SIZE} | "
                        f"M3 dist={dist_cam_to_marker:.3f}m"
                    )

                if buffer_len < self.PRECISION_BUFFER_SIZE:
                    return  # still collecting — send nothing yet

                # ── Buffer full: compute pristine locked pose ──────────────
                self.get_logger().info(
                    f"🔒 {self.PRECISION_BUFFER_SIZE} frames collected — locking final goal...")

                x_vals    = sorted([e[0] for e in self.precision_buffer])
                y_vals    = sorted([e[1] for e in self.precision_buffer])
                clean_x   = sum(x_vals[2:-2]) / (self.PRECISION_BUFFER_SIZE - 4)
                clean_y   = sum(y_vals[2:-2]) / (self.PRECISION_BUFFER_SIZE - 4)
                sin_sum   = sum(math.sin(e[2]) for e in self.precision_buffer[2:-2])
                cos_sum   = sum(math.cos(e[2]) for e in self.precision_buffer[2:-2])
                clean_yaw = math.atan2(sin_sum, cos_sum)

                # Lock immediately — no more detections processed after this
                self.docking_locked = True

                goal_x   = clean_x + self.APPROACH_DIST_PRECISION * math.cos(clean_yaw)
                goal_y   = clean_y + self.APPROACH_DIST_PRECISION * math.sin(clean_yaw)
                goal_yaw = clean_yaw + math.pi

                self.get_logger().info(
                    f"🔒 FINAL DOCKING GOAL: box=({clean_x:.3f}, {clean_y:.3f}) | "
                    f"goal=({goal_x:.3f}, {goal_y:.3f}) | "
                    f"yaw={math.degrees(goal_yaw):.1f}°"
                )

                self.publish_box_pose(clean_x, clean_y, clean_yaw)
                self.publish_debug_axes(clean_x, clean_y, clean_yaw)

                # ── CORRECT ORDER ──────────────────────────────────────────
                # 1. Send goal FIRST — Nav2 BT must still be active to receive it
                self._send_final_goal(goal_x, goal_y, goal_yaw, clean_x, clean_y)

                # 2. Reduce speed AFTER goal sent — robot drives slowly to final goal
                speed_msg             = SpeedLimit()
                speed_msg.percentage  = False
                speed_msg.speed_limit = 0.1   # slow not zero — Nav2 drives to goal
                self.speed_limit_pub.publish(speed_msg)

                # 3. Shrink footprint ONCE — authorises robot to enter box
                self.shrink_footprint()
                self.footprint_shrunk = True
                return

            # ==============================================================
            # PHASE 1 — ROUGH APPROACH BUFFERING
            # Collects ROUGH_BUFFER_SIZE stable frames then sends ONE goal
            # After rough_goal_sent=True this section is completely silent
            # ==============================================================

            # Once rough goal is sent, only publish box pose for RViz
            if self.rough_goal_sent:
                self.publish_box_pose(curr_x, curr_y, curr_yaw)
                return

            current_time = self.get_clock().now().nanoseconds

            with self._state_lock:
                # Clear rough buffer if robot moved >5cm
                # Prevents mixing far-away estimates with close ones
                robot_moved = math.hypot(t.x - self.last_robot_x,
                                         t.y - self.last_robot_y)
                if robot_moved > 0.05:
                    self.rough_buffer.clear()
                    self.last_robot_x = t.x
                    self.last_robot_y = t.y

                # Clear rough buffer if marker was lost >1s
                if (current_time - self.last_detection_time.get(m_id, 0)) > 1e9:
                    self.rough_buffer.clear()

                self.last_detection_time[m_id] = current_time
                self.rough_buffer.append((curr_x, curr_y, curr_yaw))
                rough_len = len(self.rough_buffer)

            # Log rough buffering at 1Hz
            if (now_ns - self.last_log_time.get('rough_buf', 0)) > 1e9:
                self.last_log_time['rough_buf'] = now_ns
                self.get_logger().info(
                    f"🔄 Rough buffer: {rough_len}/{self.ROUGH_BUFFER_SIZE} | "
                    f"M{m_id} dist={dist_cam_to_marker:.3f}m"
                )

            if rough_len < self.ROUGH_BUFFER_SIZE:
                return  # still collecting — send nothing yet

            # ── Buffer full: compute stable rough estimate ─────────────────
            self.get_logger().info(
                f"✅ {self.ROUGH_BUFFER_SIZE} frames collected — sending rough goal...")

            x_vals  = sorted([e[0] for e in self.rough_buffer])
            y_vals  = sorted([e[1] for e in self.rough_buffer])
            avg_x   = sum(x_vals[2:-2]) / (self.ROUGH_BUFFER_SIZE - 4)
            avg_y   = sum(y_vals[2:-2]) / (self.ROUGH_BUFFER_SIZE - 4)
            sin_sum = sum(math.sin(e[2]) for e in self.rough_buffer[2:-2])
            cos_sum = sum(math.cos(e[2]) for e in self.rough_buffer[2:-2])
            avg_yaw = math.atan2(sin_sum, cos_sum)

            goal_x   = avg_x + self.APPROACH_DIST_ROUGH * math.cos(avg_yaw)
            goal_y   = avg_y + self.APPROACH_DIST_ROUGH * math.sin(avg_yaw)
            goal_yaw = avg_yaw + math.pi

            self.get_logger().info(
                f"📍 ROUGH APPROACH GOAL: box=({avg_x:.3f}, {avg_y:.3f}) | "
                f"goal=({goal_x:.3f}, {goal_y:.3f}) | "
                f"yaw={math.degrees(goal_yaw):.1f}°"
            )

            self.publish_box_pose(avg_x, avg_y, avg_yaw)
            self.publish_debug_axes(avg_x, avg_y, avg_yaw)

            # GOAL 1 — rough approach via NavigateToPose action
            self._send_initial_goal(goal_x, goal_y, goal_yaw, avg_x, avg_y)

        except Exception as e:
            self.get_logger().warn(f"solve_box_pose error: {e}")

    # ------------------------------------------------------------------
    # GOAL SENDING — each function called exactly once
    # ------------------------------------------------------------------

    def _send_initial_goal(self, goal_x, goal_y, goal_yaw, box_x, box_y):
        """GOAL 1 — rough approach. Sent once via NavigateToPose action."""
        goal_msg = self._make_pose_stamped(goal_x, goal_y, goal_yaw)
        self._publish_mask(box_x, box_y, goal_msg.pose.orientation)
        self.rviz_goal_pub.publish(goal_msg)

        if self.send_nav2_goal(goal_msg):
            with self._state_lock:
                self.navigation_started = True
                self.rough_goal_sent    = True
            self.get_logger().info(
                f"🚀 GOAL 1 SENT (rough approach): ({goal_x:.3f}, {goal_y:.3f})"
            )
        else:
            # Nav2 not ready — clear buffer so we retry on next detection
            self.rough_buffer.clear()
            self.get_logger().warn("Nav2 not ready — rough goal will retry.")

    def _send_final_goal(self, goal_x, goal_y, goal_yaw, box_x, box_y):
        """GOAL 2 — precision docking. Sent as a fresh NavigateToPose action.
        Called BEFORE setting speed limit — Nav2 must still be active."""
        goal_msg = self._make_pose_stamped(goal_x, goal_y, goal_yaw)
        self._publish_mask(box_x, box_y, goal_msg.pose.orientation)
        self.rviz_goal_pub.publish(goal_msg)

        # Send as fresh NavigateToPose — works even if previous action completed
        self.send_nav2_goal(goal_msg)

        self.get_logger().info(
            f"🎯 GOAL 2 SENT (final docking): ({goal_x:.3f}, {goal_y:.3f})"
        )

    def _make_pose_stamped(self, x, y, yaw):
        msg                 = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = x
        msg.pose.position.y = y
        q                   = tft.quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        return msg

    def _publish_mask(self, box_x, box_y, orientation):
        """Publish box centre to goal_masking_node — punches LiDAR hole."""
        mask_msg                  = PoseStamped()
        mask_msg.header.stamp     = self.get_clock().now().to_msg()
        mask_msg.header.frame_id  = "odom"
        mask_msg.pose.position.x  = box_x
        mask_msg.pose.position.y  = box_y
        mask_msg.pose.orientation = orientation
        self.mask_pub.publish(mask_msg)

    def send_nav2_goal(self, pose_stamped):
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("Nav2 not ready yet.")
            return False
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 not available!")
            return False
        goal_msg      = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        self.nav_client.send_goal_async(goal_msg)
        return True

    # ------------------------------------------------------------------
    # VISUALISATION
    # ------------------------------------------------------------------

    def publish_box_pose(self, x, y, yaw):
        msg                 = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = x
        msg.pose.position.y = y
        q                   = tft.quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.box_pub.publish(msg)

    def publish_debug_axes(self, x, y, yaw):
        marker_array = MarkerArray()

        # Red arrow — box forward direction
        m1                    = Marker()
        m1.header.frame_id    = "odom"
        m1.header.stamp       = self.get_clock().now().to_msg()
        m1.id                 = 0
        m1.type               = Marker.ARROW
        m1.action             = Marker.ADD
        m1.pose.position.x    = x
        m1.pose.position.y    = y
        m1.pose.position.z    = 0.0
        q                     = tft.quaternion_from_euler(0, 0, yaw)
        m1.pose.orientation.x = q[0]; m1.pose.orientation.y = q[1]
        m1.pose.orientation.z = q[2]; m1.pose.orientation.w = q[3]
        m1.scale.x = 0.5; m1.scale.y = 0.05; m1.scale.z = 0.05
        m1.color.r = 1.0; m1.color.a = 1.0
        marker_array.markers.append(m1)

        # Green arrow — box left direction
        m2                    = Marker()
        m2.header.frame_id    = "odom"
        m2.header.stamp       = self.get_clock().now().to_msg()
        m2.id                 = 1
        m2.type               = Marker.ARROW
        m2.action             = Marker.ADD
        m2.pose.position.x    = x
        m2.pose.position.y    = y
        m2.pose.position.z    = 0.0
        q2                    = tft.quaternion_from_euler(0, 0, yaw + math.pi/2)
        m2.pose.orientation.x = q2[0]; m2.pose.orientation.y = q2[1]
        m2.pose.orientation.z = q2[2]; m2.pose.orientation.w = q2[3]
        m2.scale.x = 0.3; m2.scale.y = 0.05; m2.scale.z = 0.05
        m2.color.g = 1.0; m2.color.a = 1.0
        marker_array.markers.append(m2)

        self.viz_pub.publish(marker_array)

    def shrink_footprint(self):
        """Shrink robot footprint to 4cm so Nav2 allows entry into the box."""
        poly        = Polygon()
        tiny_points = [[-0.04, -0.04], [-0.04, 0.04], [0.04, 0.04], [0.04, -0.04]]
        for pt_coords in tiny_points:
            pt   = Point32()
            pt.x = float(pt_coords[0])
            pt.y = float(pt_coords[1])
            pt.z = 0.0
            poly.points.append(pt)
        self.local_footprint_pub.publish(poly)
        self.global_footprint_pub.publish(poly)
        self.get_logger().info("🛡️ Footprint shrunk — authorizing entry into box.")


def main():
    rclpy.init()
    node     = BoxEstimator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()