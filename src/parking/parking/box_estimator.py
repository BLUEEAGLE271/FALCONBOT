#!/usr/bin/env python3
"""
box_estimator.py — AprilTag → Box Centre Pose Publisher
=======================================================
Pure pose computation node. Subscribes to /apriltag/marker_{0-3},
runs the full transform chain (odom→camera→marker→box_centre), and
publishes the result continuously to /box_center_pose.

All navigation logic and state machine live in docking_supervisor.py.
"""

import math
import threading

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

import tf_transformations as tft
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener


# ── Homogeneous matrix helpers ────────────────────────────────────────────────

def Trans_z(dz):
    M = np.eye(4); M[2, 3] = dz; return M

def Rx(deg):
    r = math.radians(deg); c, s = math.cos(r), math.sin(r)
    M = np.eye(4); M[1,1], M[1,2] = c, -s; M[2,1], M[2,2] = s, c; return M

def Ry(deg):
    r = math.radians(deg); c, s = math.cos(r), math.sin(r)
    M = np.eye(4); M[0,0], M[0,2] = c, s; M[2,0], M[2,2] = -s, c; return M

def Rz(deg):
    r = math.radians(deg); c, s = math.cos(r), math.sin(r)
    M = np.eye(4); M[0,0], M[0,1] = c, -s; M[1,0], M[1,1] = s, c; return M


# ── Node ──────────────────────────────────────────────────────────────────────

class BoxEstimator(Node):

    BOX_LENGTH  = 0.25
    BOX_WIDTH   = 0.20
    TARGET_IDS  = [0, 1, 2, 3]

    def __init__(self):
        super().__init__('box_estimator')

        # Marker-3 priority gate
        self._last_det_ns: dict[int, int] = {}
        self._lock = threading.Lock()

        # Build marker→box-centre transforms once
        self._marker_to_box: dict[int, np.ndarray] = {}
        self._setup_geometry()

        self._tf_buffer   = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        cb = ReentrantCallbackGroup()
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._box_pub = self.create_publisher(PoseStamped, '/box_center_pose', 10)
        self._viz_pub = self.create_publisher(MarkerArray, '/box_debug_axes',  10)

        for m_id in self.TARGET_IDS:
            self.create_subscription(
                PoseStamped,
                f'/apriltag/marker_{m_id}',
                lambda msg, mid=m_id: self._on_marker(msg, mid),
                qos,
                callback_group=cb,
            )

        self.get_logger().info('BoxEstimator ready — publishing /box_center_pose')

    def _setup_geometry(self):
        L, W = self.BOX_LENGTH, self.BOX_WIDTH
        self._marker_to_box[0] = Trans_z(L / 2)         @ Rx(90)
        self._marker_to_box[1] = Trans_z(L / 2)         @ Ry(180) @ Rx(90)
        self._marker_to_box[2] = Trans_z(W / 2 + 0.003) @ Ry(90)  @ Rx(90)
        self._marker_to_box[3] = Trans_z(-W / 2)        @ Ry(-90) @ Rx(90)

    def _on_marker(self, msg: PoseStamped, m_id: int):
        # Marker-3 priority: suppress others if marker 3 seen within 500 ms
        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            if m_id != 3 and (now_ns - self._last_det_ns.get(3, 0)) < 500_000_000:
                return
            self._last_det_ns[m_id] = now_ns

        try:
            pose = msg.pose
            t_cam_marker = tft.concatenate_matrices(
                tft.translation_matrix([pose.position.x,
                                        pose.position.y,
                                        pose.position.z]),
                tft.quaternion_matrix([pose.orientation.x, pose.orientation.y,
                                       pose.orientation.z, pose.orientation.w]),
            )

            try:
                tf_s = self._tf_buffer.lookup_transform(
                    'odom', msg.header.frame_id, msg.header.stamp,
                    rclpy.duration.Duration(seconds=0.3),
                )
            except Exception:
                tf_s = self._tf_buffer.lookup_transform(
                    'odom', msg.header.frame_id, rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.1),
                )

            tr = tf_s.transform.translation
            ro = tf_s.transform.rotation
            t_odom_cam = tft.concatenate_matrices(
                tft.translation_matrix([tr.x, tr.y, tr.z]),
                tft.quaternion_matrix([ro.x, ro.y, ro.z, ro.w]),
            )

            t_odom_box = t_odom_cam @ t_cam_marker @ self._marker_to_box[m_id]
            x   = t_odom_box[0, 3]
            y   = t_odom_box[1, 3]
            yaw = math.atan2(t_odom_box[1, 0], t_odom_box[0, 0])

        except Exception as e:
            self.get_logger().error(f'TF error: {e}', throttle_duration_sec=1.0)
            return

        self._publish_pose(x, y, yaw)
        self._publish_axes(x, y, yaw)

    def _publish_pose(self, x: float, y: float, yaw: float):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = x
        msg.pose.position.y = y
        q = tft.quaternion_from_euler(0.0, 0.0, yaw)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = q
        self._box_pub.publish(msg)

    def _publish_axes(self, x: float, y: float, yaw: float):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def arrow(uid, yaw_off, r, g, b, sx):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp    = stamp
            m.id, m.type, m.action = uid, Marker.ARROW, Marker.ADD
            m.pose.position.x, m.pose.position.y = x, y
            q = tft.quaternion_from_euler(0.0, 0.0, yaw + yaw_off)
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = q
            m.scale.x, m.scale.y, m.scale.z = sx, 0.05, 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            return m

        arr.markers.append(arrow(0, 0.0,          1.0, 0.0, 0.0, 0.5))  # X red
        arr.markers.append(arrow(1, math.pi / 2,  0.0, 1.0, 0.0, 0.3))  # Y green
        self._viz_pub.publish(arr)


def main():
    rclpy.init()
    node = BoxEstimator()
    executor = MultiThreadedExecutor(num_threads=2)
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
