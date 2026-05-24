#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf_transformations as tft
import math
import numpy as np
import threading
from collections import deque
 
try:
    from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
    ISAAC_ROS_AVAILABLE = True
except ImportError:
    ISAAC_ROS_AVAILABLE = False
    print("WARNING: isaac_ros_apriltag_interfaces not found. "
          "Install isaac_ros_apriltag first.")
 
 
# ── Configuration ─────────────────────────────────────────────────────
TARGET_IDS            = [0, 1, 2, 3]
PRECISION_MARKER_ID   = 3
PRECISION_TRIGGER_DIST = 0.7     # metres — raw camera distance to trigger precision

 
# EMA filter alphas — lower = smoother but more lag
ALPHA_DEFAULT = 0.3   # markers 0, 1, 2
ALPHA_MARKER3 = 0.1   # marker 3 — extra smooth (precision docking marker)
 
# Filter reset if marker unseen for this many seconds
FILTER_RESET_TIMEOUT = 0.5
 
# Reject quaternion jump larger than this (degrees) — catches 180° flips
FLIP_REJECT_DEG = 90.0
 
# Process every N detections — reduces CPU load
PROCESS_EVERY_N = 1   # AprilTag is already GPU-accelerated so no need to skip
# ─────────────────────────────────────────────────────────────────────
 
 
class MarkerFilter:
    def __init__(self):
        self.tvec        = None      # [x, y, z] in camera frame
        self.quat        = None      # [x, y, z, w]
        self.initialized = False
        self.last_seen   = 0.0       # time.time()
 
 
class AprilTagTracker(Node):
    def __init__(self):
        super().__init__('apriltag_tracker')
 
        if not ISAAC_ROS_AVAILABLE:
            self.get_logger().error(
                "isaac_ros_apriltag_interfaces not installed! "
                "Run: sudo apt install ros-humble-isaac-ros-apriltag")
            return
 
        # Per-marker EMA filters
        self._filters = {i: MarkerFilter() for i in TARGET_IDS}
        self._lock    = threading.Lock()
        self._det_count = 0
 
        # TF
        self.tf_buffer    = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener  = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
 
        be_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
 
        # Subscribe to isaac_ros_apriltag output
        self.det_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self._detection_callback,
            be_qos
        )
 
        # Per-marker pose publishers (same interface as old /aruco/marker_N)
        self._marker_pubs = {}
        for m_id in TARGET_IDS:
            self._marker_pubs[m_id] = self.create_publisher(
                PoseStamped,
                f'/apriltag/marker_{m_id}',
                10
            )
 
        # Debug goal pose — visualisation only, not sent to Nav2
 
        # Debug axes
        self.axes_pub = self.create_publisher(
            MarkerArray, '/apriltag/debug_axes', 10)
 
 
        # Track best marker for debug goal
        self._best_marker_id   = None
        self._best_marker_tvec = None
        self._best_marker_quat = None
        self._best_marker_dist = float('inf')
        self._best_marker_time = 0.0
        self._last_log_time    = {}
 
        self.get_logger().info(
            f"AprilTag Tracker ready | "
            f"Markers: {TARGET_IDS} | "
            f"Precision trigger: M{PRECISION_MARKER_ID} within {PRECISION_TRIGGER_DIST}m | "
            f"Debug goal: /debug_goal_pose (RViz only)"
        )
 
    # ------------------------------------------------------------------
    # DETECTION CALLBACK
    # ------------------------------------------------------------------
 
    def _detection_callback(self, msg: AprilTagDetectionArray):
        self._det_count += 1
 
        if self._det_count % PROCESS_EVERY_N != 0:
            return
 
        now_sec = self.get_clock().now().nanoseconds / 1e9
 
        # Reset best marker tracking each cycle
        best_id   = None
        best_tvec = None
        best_quat = None
        best_dist = float('inf')
 
        for det in msg.detections:
            m_id = det.id
            if m_id not in TARGET_IDS:
                continue
 
            # Extract pose from isaac_ros_apriltag detection
            # pose is PoseWithCovarianceStamped in camera frame
            pos = det.pose.pose.pose.position
            ori = det.pose.pose.pose.orientation
 
            tvec = np.array([pos.x, pos.y, pos.z])
            quat = np.array([ori.x, ori.y, ori.z, ori.w])
            dist = float(np.linalg.norm(tvec))
 
            with self._lock:
                f = self._filters[m_id]
                alpha = ALPHA_MARKER3 if m_id == PRECISION_MARKER_ID else ALPHA_DEFAULT
 
                # Filter reset if marker was lost
                gap = now_sec - f.last_seen
                if f.initialized and gap > FILTER_RESET_TIMEOUT:
                    self.get_logger().info(
                        f"M{m_id}: Lost for {gap:.2f}s — resetting filter")
                    f.initialized = False
 
                f.last_seen = now_sec
 
                if not f.initialized:
                    # First detection or post-reset — accept directly
                    f.tvec        = tvec.copy()
                    f.quat        = quat.copy()
                    f.initialized = True
 
                else:
                    # Flip rejection — detect 180° quaternion jump
                    q_prev = f.quat
                    # Dot product between quaternions
                    dot = abs(float(np.dot(q_prev, quat)))
                    dot = min(1.0, dot)
                    angle_rad = 2.0 * math.acos(dot)
                    flip_threshold = FLIP_REJECT_DEG * math.pi / 180.0
 
                    if angle_rad > flip_threshold:
                        self.get_logger().warn(
                            f"M{m_id}: Flip rejected "
                            f"({math.degrees(angle_rad):.1f}° > {FLIP_REJECT_DEG}°) "
                            f"— holding last pose",
                            throttle_duration_sec=0.5
                        )
                        # Publish last known good pose without updating filter
                        self._publish_marker(
                            f.tvec, f.quat,
                            msg.header, m_id
                        )
                        self._publish_tf(f.tvec, f.quat, msg.header, m_id)
                        continue
 
                    # Valid — apply EMA filter
                    f.tvec = alpha * tvec + (1.0 - alpha) * f.tvec
 
                    # SLERP for quaternion
                    # Ensure same hemisphere
                    if np.dot(f.quat, quat) < 0:
                        quat = -quat
                    f.quat = tft.quaternion_slerp(f.quat, quat, alpha)
                    f.quat = f.quat / np.linalg.norm(f.quat)
 
                filtered_tvec = f.tvec.copy()
                filtered_quat = f.quat.copy()
 
            # Publish filtered pose
            self._publish_marker(filtered_tvec, filtered_quat, msg.header, m_id)
            self._publish_tf(filtered_tvec, filtered_quat, msg.header, m_id)
            self._publish_axes(filtered_tvec, filtered_quat,msg.header, m_id)
 
            # Track best marker for debug goal
            # Prefer marker 3 if within precision trigger distance
            if m_id == PRECISION_MARKER_ID and dist <= PRECISION_TRIGGER_DIST:
                best_id   = m_id
                best_tvec = filtered_tvec
                best_quat = filtered_quat
                best_dist = dist
            elif best_id != PRECISION_MARKER_ID and dist < best_dist:
                best_id   = m_id
                best_tvec = filtered_tvec
                best_quat = filtered_quat
                best_dist = dist
 
            # 1Hz log per marker
            now_ns = self.get_clock().now().nanoseconds
            if (now_ns - self._last_log_time.get(m_id, 0)) > 1e9:
                self._last_log_time[m_id] = now_ns
                self.get_logger().info(
                    f"[M{m_id}] dist={dist:.3f}m | "
                    f"x={filtered_tvec[0]:.3f} y={filtered_tvec[1]:.3f} "
                    f"z={filtered_tvec[2]:.3f}"
                )
 
        # Update best marker
        with self._lock:
            if best_id is not None:
                self._best_marker_id   = best_id
                self._best_marker_tvec = best_tvec
                self._best_marker_quat = best_quat
                self._best_marker_dist = best_dist
                self._best_marker_time = now_sec
 
    # ------------------------------------------------------------------
    # DEBUG GOAL POSE — published at 5Hz, visible in RViz always
    # ------------------------------------------------------------------
 
    def _publish_debug_goal(self):
        """
        Computes approach goal pose from best visible marker and
        publishes to /debug_goal_pose for RViz visualisation.
        NOT sent to Nav2. Shows where the robot would drive to.
 
        Logic:
          - If marker 3 within 0.7m → use precision approach (0.15m offset)
          - Otherwise use rough approach (0.5m offset) from any marker
        """
        with self._lock:
            m_id   = self._best_marker_id
            tvec   = self._best_marker_tvec
            quat   = self._best_marker_quat
            dist   = self._best_marker_dist
            t_seen = self._best_marker_time
 
        if m_id is None or tvec is None:
            return
 
        # Clear stale data after 2 seconds
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - t_seen > 2.0:
            return
 
        # Determine approach distance
        if m_id == PRECISION_MARKER_ID and dist <= PRECISION_TRIGGER_DIST:
            approach = APPROACH_DIST_PREC
            mode     = "PRECISION"
        else:
            approach = APPROACH_DIST_ROUGH
            mode     = "ROUGH"
 
        # Build camera→marker transform
        t_cam_marker = tft.concatenate_matrices(
            tft.translation_matrix(tvec),
            tft.quaternion_matrix(quat)
        )
 
        # Goal is approach_dist along the marker's -Z axis (facing the camera)
        # In marker frame: goal is at [0, 0, -approach] offset from marker
        goal_in_marker = np.array([0.0, 0.0, -approach, 1.0])
        goal_in_cam    = t_cam_marker @ goal_in_marker
 
        # Now transform from camera frame to odom frame via TF
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                'odom',
                'camera_optical',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1)
            )
        except Exception:
            return  # TF not available yet
 
        t  = tf_stamped.transform.translation
        r  = tf_stamped.transform.rotation
        t_odom_cam = tft.concatenate_matrices(
            tft.translation_matrix([t.x, t.y, t.z]),
            tft.quaternion_matrix([r.x, r.y, r.z, r.w])
        )
 
        # Transform marker pose to odom
        t_odom_marker = t_odom_cam @ t_cam_marker
 
        # Goal position in odom
        goal_odom = t_odom_cam @ goal_in_cam
 
        # Yaw: robot should face the marker — opposite of marker's forward axis
        # Extract yaw from marker rotation in odom frame
        _, _, yaw = tft.euler_from_matrix(t_odom_marker)
        goal_yaw  = yaw + math.pi   # face the marker
 
        # Publish debug goal
        msg                   = PoseStamped()
        msg.header.stamp      = self.get_clock().now().to_msg()
        msg.header.frame_id   = 'odom'
        msg.pose.position.x   = float(goal_odom[0])
        msg.pose.position.y   = float(goal_odom[1])
        msg.pose.position.z   = 0.0
        q = tft.quaternion_from_euler(0, 0, goal_yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.debug_goal_pub.publish(msg)
 
        # Log at 1Hz
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_log_time.get('debug', 0)) > 1e9:
            self._last_log_time['debug'] = now_ns
            self.get_logger().info(
                f"🎯 Debug goal [{mode}] M{m_id} "
                f"dist={dist:.3f}m → "
                f"goal=({goal_odom[0]:.3f}, {goal_odom[1]:.3f}) "
                f"yaw={math.degrees(goal_yaw):.1f}°"
            )
 
    # ------------------------------------------------------------------
    # PUBLISHERS
    # visual
 
    def _publish_marker(self, tvec, quat, header, m_id):
        msg                   = PoseStamped()
        msg.header            = header
        msg.pose.position.x   = float(tvec[0])
        msg.pose.position.y   = float(tvec[1])
        msg.pose.position.z   = float(tvec[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        self._marker_pubs[m_id].publish(msg)
 
    def _publish_tf(self, tvec, quat, header, m_id):
        t                          = TransformStamped()
        t.header.stamp             = header.stamp
        t.header.frame_id          = header.frame_id
        t.child_frame_id           = f'apriltag_{m_id}'
        t.transform.translation.x  = float(tvec[0])
        t.transform.translation.y  = float(tvec[1])
        t.transform.translation.z  = float(tvec[2])
        t.transform.rotation.x     = float(quat[0])
        t.transform.rotation.y     = float(quat[1])
        t.transform.rotation.z     = float(quat[2])
        t.transform.rotation.w     = float(quat[3])
        self.tf_broadcaster.sendTransform(t)
 
    def _publish_axes(self, tvec, quat,header, m_id):
        """Publish debug axes arrows in camera frame for RViz."""
        ma = MarkerArray()
 
        for i, (color, yaw_offset) in enumerate([(
                (1.0, 0.0, 0.0), 0.0),   # red = forward
                ((0.0, 1.0, 0.0), math.pi/2)]):  # green = left
            m                    = Marker()
            m.header.frame_id    = header.frame_id
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = f'apriltag_{m_id}'
            m.id                 = m_id * 10 + i
            m.type               = Marker.ARROW
            m.action             = Marker.ADD
            m.pose.position.x    = float(tvec[0])
            m.pose.position.y    = float(tvec[1])
            m.pose.position.z    = float(tvec[2])
            m.pose.orientation.x = float(quat[0])
            m.pose.orientation.y = float(quat[1])
            m.pose.orientation.z = float(quat[2])
            m.pose.orientation.w = float(quat[3])
            m.scale.x = 0.1; m.scale.y = 0.01; m.scale.z = 0.01
            m.color.r = color[0]; m.color.g = color[1]
            m.color.b = color[2]; m.color.a = 1.0
            ma.markers.append(m)
 
        self.axes_pub.publish(ma)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = AprilTagTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
 
 