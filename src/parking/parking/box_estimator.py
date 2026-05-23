#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Polygon, Point32
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.msg import SpeedLimit
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft
import numpy as np
import math
import threading
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


# ---------------------------------------------------------------------------
# Homogeneous matrix primitives — intrinsic (body-fixed), right-hand rule
# ---------------------------------------------------------------------------

def Trans_z(dz):
    M = np.eye(4)
    M[2, 3] = dz
    return M

def Rx(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    M = np.eye(4)
    M[1, 1], M[1, 2] =  c, -s
    M[2, 1], M[2, 2] =  s,  c
    return M

def Ry(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    M = np.eye(4)
    M[0, 0], M[0, 2] =  c,  s
    M[2, 0], M[2, 2] = -s,  c
    return M


class BoxEstimator(Node):
    def __init__(self):
        super().__init__('box_estimator')

        # --- CONFIGURATION ---
        self.BOX_LENGTH = 0.25
        self.BOX_WIDTH  = 0.20
        self.APPROACH_DIST_ROUGH     = 0.1
        self.APPROACH_DIST_PRECISION = 0.15

        self.target_ids = [0, 1, 2, 3]

        # --- STATE MACHINE ---
        self.mission_active     = False
        self.docking_locked     = False
        self.precision_mode     = False
        self.rough_goal_sent    = False
        self.navigation_started = False
        self.footprint_shrunk   = False

        self.ROUGH_BUFFER_SIZE  = 20
        self.PRECISION_BUFFER_SIZE = 20
        self.PRECISION_TRIGGER_DIST = 1.0

        self.rough_buffer     = []
        self.precision_buffer = []

        self.last_detection_time = {}
        self.last_log_time       = {}
        self.last_robot_x        = 0.0
        self.last_robot_y        = 0.0

        # --- GEOMETRY ---
        self.marker_to_box = {}
        self.setup_box_geometry()

        # --- ROS SETUP ---
        self.cb_group    = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.box_pub              = self.create_publisher(PoseStamped, '/box_center_pose', 10)
        self.viz_pub              = self.create_publisher(MarkerArray, '/box_debug_axes', 10)
        self.rviz_goal_pub        = self.create_publisher(PoseStamped, '/visual_target_goal', 10)
        self.mask_pub             = self.create_publisher(PoseStamped, '/parking_goal', 10)
        self.speed_limit_pub      = self.create_publisher(SpeedLimit,  '/speed_limit', 10)
        self.local_footprint_pub  = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.global_footprint_pub = self.create_publisher(Polygon, '/global_costmap/footprint', 10)

        self.tf_buffer   = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        marker_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subs = []
        for m_id in self.target_ids:
            sub = self.create_subscription(
                PoseStamped,
                f'/apriltag/marker_{m_id}',
                lambda msg, mid=m_id: self.detection_callback(msg, mid),
                marker_qos,
                callback_group=self.cb_group
            )
            self.subs.append(sub)

        self.start_sub = self.create_subscription(
            Bool, '/start_mission', self.start_callback, 10, callback_group=self.cb_group
        )

        self.get_logger().info("Box Estimator Ready.")

    # ------------------------------------------------------------------
    # GEOMETRY DEFINITION
    # ------------------------------------------------------------------
    def setup_box_geometry(self):
        L = self.BOX_LENGTH
        W = self.BOX_WIDTH

        # Tag 0: translate along z by L/2, then rotate 90° about x
        self.marker_to_box[0] = Trans_z(L / 2) @ Rx(90)

        # Tag 1: translate along z by L/2, rotate 180° about y, then 90° about x
        self.marker_to_box[1] = Trans_z(L / 2) @ Ry(180) @ Rx(90)

        # Tag 2: translate along z by W/2 + 0.003, rotate 90° about y, then 90° about x
        self.marker_to_box[2] = Trans_z(W / 2 + 0.003) @ Ry(90) @ Rx(90)

        # Tag 3: translate along z by -W/2, rotate -90° about y, then 90° about x
        self.marker_to_box[3] = Trans_z(-W / 2) @ Ry(-90) @ Rx(90)

    # ------------------------------------------------------------------
    # CALLBACKS & CORE MATH
    # ------------------------------------------------------------------
    def start_callback(self, msg):
        self.mission_active = msg.data
        if self.mission_active:
            self.get_logger().info("MISSION ACTIVE")
        else:
            self.get_logger().info("MISSION DEACTIVATED")

    def detection_callback(self, msg, m_id):
        self.get_logger().info(f"Detected AprilTag {m_id}!", throttle_duration_sec=1.0)

        if self.docking_locked:
            return

        now_ns = self.get_clock().now().nanoseconds
        m3_age_ns = now_ns - self.last_detection_time.get(3, 0)

        # Prioritize Marker 3
        if m_id != 3 and m3_age_ns < 500_000_000:
            return

        with self._state_lock:
            self.last_detection_time[m_id] = now_ns

        try:
            pose = msg.pose

            # 1. t_cam_marker — marker pose expressed in the camera frame
            t_cam_marker = tft.concatenate_matrices(
                tft.translation_matrix([pose.position.x, pose.position.y, pose.position.z]),
                tft.quaternion_matrix([pose.orientation.x, pose.orientation.y,
                                       pose.orientation.z, pose.orientation.w])
            )

            # 2. t_map_cam — odom-to-camera transform via TF
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    'odom', msg.header.frame_id, msg.header.stamp,
                    rclpy.duration.Duration(seconds=0.3)
                )
            except Exception:
                tf_stamped = self.tf_buffer.lookup_transform(
                    'odom', msg.header.frame_id, rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.1)
                )

            t = tf_stamped.transform.translation
            r = tf_stamped.transform.rotation
            t_map_cam = tft.concatenate_matrices(
                tft.translation_matrix([t.x, t.y, t.z]),
                tft.quaternion_matrix([r.x, r.y, r.z, r.w])
            )

            # 3. t_tag_boxcenter — pre-built intrinsic transform, used as-is
            t_tag_boxcenter = self.marker_to_box[m_id]

            # 4. Full chain: odom -> camera -> marker -> box center
            t_map_box = t_map_cam @ t_cam_marker @ t_tag_boxcenter

            # 5. 2D ground pose
            curr_x = t_map_box[0, 3]
            curr_y = t_map_box[1, 3]

            # 6. Pure 2D heading: project box-center local X-axis onto the map floor
            curr_yaw = math.atan2(t_map_box[1, 0], t_map_box[0, 0])

            self.publish_box_pose(curr_x, curr_y, curr_yaw)
            self.publish_debug_axes(curr_x, curr_y, curr_yaw)

            if not self.mission_active:
                return

            dist_cam_to_marker = math.sqrt(
                pose.position.x**2 + pose.position.y**2 + pose.position.z**2
            )

            self.process_state_machine(m_id, curr_x, curr_y, curr_yaw, dist_cam_to_marker, t.x, t.y)

        except Exception as e:
            self.get_logger().error(f"MATH/TF ERROR: {e}", throttle_duration_sec=1.0)

    # ------------------------------------------------------------------
    # STATE MACHINE LOGIC
    # ------------------------------------------------------------------
    def process_state_machine(self, m_id, curr_x, curr_y, curr_yaw, dist, robot_x, robot_y):
        # PHASE 2: Precision Mode Trigger
        if m_id == 3 and dist <= self.PRECISION_TRIGGER_DIST and not self.precision_mode:
            self.precision_mode = True
            self.get_logger().info("Precision Mode triggered! Collecting frames...")
            self.publish_speed_limit(0.15)

        if self.precision_mode:
            if m_id != 3:
                return

            with self._state_lock:
                self.precision_buffer.append((curr_x, curr_y, curr_yaw))
                if len(self.precision_buffer) < self.PRECISION_BUFFER_SIZE:
                    return

            self.docking_locked = True
            goal_x, goal_y, goal_yaw = self.compute_averaged_goal(
                self.precision_buffer, self.APPROACH_DIST_PRECISION
            )
            self.get_logger().info("FINAL DOCKING GOAL LOCKED.")

            self._send_nav_goal(goal_x, goal_y, goal_yaw, curr_x, curr_y)
            self.publish_speed_limit(0.1)
            self.shrink_footprint()
            return

        # PHASE 1: Rough Approach
        if self.rough_goal_sent:
            return

        with self._state_lock:
            # Clear buffer if robot moved >5 cm to avoid smeared averages
            if math.hypot(robot_x - self.last_robot_x, robot_y - self.last_robot_y) > 0.05:
                self.rough_buffer.clear()
                self.last_robot_x, self.last_robot_y = robot_x, robot_y

            self.rough_buffer.append((curr_x, curr_y, curr_yaw))
            if len(self.rough_buffer) < self.ROUGH_BUFFER_SIZE:
                return

        goal_x, goal_y, goal_yaw = self.compute_averaged_goal(
            self.rough_buffer, self.APPROACH_DIST_ROUGH
        )
        self.get_logger().info("ROUGH APPROACH GOAL READY.")

        if self._send_nav_goal(goal_x, goal_y, goal_yaw, curr_x, curr_y):
            self.rough_goal_sent = True
        else:
            self.rough_buffer.clear()

    # ------------------------------------------------------------------
    # UTILITIES
    # ------------------------------------------------------------------
    def compute_averaged_goal(self, buffer, approach_dist):
        x_vals = sorted([e[0] for e in buffer])
        y_vals = sorted([e[1] for e in buffer])
        # Strip top-2 and bottom-2 outliers
        avg_x = sum(x_vals[2:-2]) / (len(buffer) - 4)
        avg_y = sum(y_vals[2:-2]) / (len(buffer) - 4)

        sin_sum = sum(math.sin(e[2]) for e in buffer[2:-2])
        cos_sum = sum(math.cos(e[2]) for e in buffer[2:-2])
        avg_yaw = math.atan2(sin_sum, cos_sum)

        goal_x   = avg_x + approach_dist * math.cos(avg_yaw)
        goal_y   = avg_y + approach_dist * math.sin(avg_yaw)
        goal_yaw = avg_yaw + math.pi   # face the box
        return goal_x, goal_y, goal_yaw

    def _send_nav_goal(self, gx, gy, gyaw, box_x, box_y):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        q = tft.quaternion_from_euler(0, 0, gyaw)
        msg.pose.orientation.x, msg.pose.orientation.y, \
            msg.pose.orientation.z, msg.pose.orientation.w = q

        self.rviz_goal_pub.publish(msg)

        mask = PoseStamped()
        mask.header = msg.header
        mask.pose.position.x = box_x
        mask.pose.position.y = box_y
        mask.pose.orientation = msg.pose.orientation
        self.mask_pub.publish(mask)

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 server timeout!")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        self.nav_client.send_goal_async(goal_msg)
        return True

    def publish_box_pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = x
        msg.pose.position.y = y
        q = tft.quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation.x, msg.pose.orientation.y, \
            msg.pose.orientation.z, msg.pose.orientation.w = q
        self.box_pub.publish(msg)

    def publish_debug_axes(self, x, y, yaw):
        arr = MarkerArray()

        # Box X-axis (Red — approach direction)
        m1 = Marker()
        m1.header.frame_id = "odom"
        m1.header.stamp    = self.get_clock().now().to_msg()
        m1.id   = 0
        m1.type = Marker.ARROW
        m1.action = Marker.ADD
        m1.pose.position.x, m1.pose.position.y = x, y
        q = tft.quaternion_from_euler(0, 0, yaw)
        m1.pose.orientation.x, m1.pose.orientation.y, \
            m1.pose.orientation.z, m1.pose.orientation.w = q
        m1.scale.x, m1.scale.y, m1.scale.z = 0.5, 0.05, 0.05
        m1.color.r, m1.color.a = 1.0, 1.0
        arr.markers.append(m1)

        # Box Y-axis (Green — lateral)
        m2 = Marker()
        m2.header = m1.header
        m2.id     = 1
        m2.type   = Marker.ARROW
        m2.action = Marker.ADD
        m2.pose.position.x, m2.pose.position.y = x, y
        q2 = tft.quaternion_from_euler(0, 0, yaw + math.pi / 2)
        m2.pose.orientation.x, m2.pose.orientation.y, \
            m2.pose.orientation.z, m2.pose.orientation.w = q2
        m2.scale.x, m2.scale.y, m2.scale.z = 0.3, 0.05, 0.05
        m2.color.g, m2.color.a = 1.0, 1.0
        arr.markers.append(m2)

        self.viz_pub.publish(arr)

    def publish_speed_limit(self, limit):
        msg = SpeedLimit()
        msg.percentage  = False
        msg.speed_limit = float(limit)
        self.speed_limit_pub.publish(msg)

    def shrink_footprint(self):
        poly = Polygon()
        for px, py in [[-0.04, -0.04], [-0.04, 0.04], [0.04, 0.04], [0.04, -0.04]]:
            pt = Point32()
            pt.x, pt.y = px, py
            poly.points.append(pt)
        self.local_footprint_pub.publish(poly)
        self.global_footprint_pub.publish(poly)


def main():
    rclpy.init()
    node = BoxEstimator()
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
