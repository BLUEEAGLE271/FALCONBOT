#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
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
from rclpy.qos import QoSProfile,

class BoxEstimator(Node):
    def __init__(self):
        super().__init__('box_estimator')

        # --- CONFIGURATION ---
        self.BOX_LENGTH = 0.204   
        self.BOX_WIDTH  = 0.20   
        self.half_L = self.BOX_LENGTH / 2.0
        self.half_W = self.BOX_WIDTH / 2.0
        self.APPROACH_DIST = 0.4 
        
        self.target_ids = [0, 1, 2, 3]
        self.navigation_started = False 
        self.footprint_shrunk = False

        # --- GEOMETRY ---
        self.marker_transforms = {}
        self.setup_box_geometry()

        # --- FILTERING ---
        self.filter_size = 10   # Increased from 5 for smoother odom-frame estimates
        self.pose_history = {} 

        self.precision_mode = False
        self.docking_locked = False
        self.precision_buffer = []
        self.last_detection_time = {}

        # --- DEBUG LOGGING ---
        self.last_log_time = {}

        # --- Nav2 retry throttle ---
        self.last_nav2_attempt_ns = 0

        # --- ROS SETUP ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.update_pub = self.create_publisher(PoseStamped, '/goal_update', 10)
        self.mask_pub = self.create_publisher(PoseStamped, '/parking_goal', 10)
        self.box_pub = self.create_publisher(PoseStamped, '/box_center_pose', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/box_debug_axes', 10)
        self.speed_limit_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.local_footprint_pub = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.global_footprint_pub = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        marker_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # drop rather than queue
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1  # only ever process the most recent detection
        )


        self.subs = []
        for m_id in self.target_ids:
            topic_name = f'/aruco/marker_{m_id}'
            sub = self.create_subscription(
                PoseStamped, 
                topic_name, 
                lambda msg, mid=m_id: self.detection_callback(msg, mid), 
                marker_qos
            )
            self.subs.append(sub)

        self.mission_active = False
        self.start_sub = self.create_subscription(
            Bool, 
            '/start_mission', 
            self.start_callback, 
            10
        )
        
        self.get_logger().info("Box Estimator Ready [DEBUG MODE]")
    
    def detection_callback(self, msg, m_id):
        if not self.mission_active:
            return
        if self.docking_locked:
            return
        self.solve_box_pose(msg, m_id)
    
    def start_callback(self, msg):
        if msg.data is True:
            self.get_logger().info("✅ MISSION ACTIVE: Box Estimator now processing markers.")
            self.mission_active = True
        else:
            self.get_logger().info("🛑 MISSION DEACTIVATED: Stopping goal updates.")
            self.mission_active = False

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
            [0.0, self.half_W, 0.2],
            [-1, 0, 0], [0, 0, 1], [0, 1, 0]
        )

        # Marker 0 — Right Face (-Y)
        self.marker_transforms[0] = make_mat(
            [0.0, -self.half_W, 0.2], 
            [1, 0, 0], [0, 0, 1], [0, -1, 0]
        )

        # Marker 2 — Back Face (-X)
        self.marker_transforms[2] = make_mat(
            [-self.half_L, 0.0, 0.2],
            [0, 1, 0], [0, 0, 1], [1, 0, 0]
        )

        # Marker 3 — Front Face (+X)
        self.marker_transforms[3] = make_mat(
            [self.half_L - 0.06, 0.0, 0.2],
            [0, -1, 0], [0, 0, 1], [-1, 0, 0]
        )

    def solve_box_pose(self, pose_msg, m_id):
        try:
            # 1. Camera -> Marker (from ArUco detection)
            pose = pose_msg.pose   # renamed from 'p' to avoid shadowing loop vars below
            t_cam_marker = tft.concatenate_matrices(
                tft.translation_matrix([pose.position.x, pose.position.y, pose.position.z]), 
                tft.quaternion_matrix([pose.orientation.x, pose.orientation.y,
                                       pose.orientation.z, pose.orientation.w])
            )

            # 2. Odom -> Camera
            # ----------------------------------------------------------------
            # FIX 1: Use image header timestamp for TF lookup first.
            # This matches the robot's position at the MOMENT the frame was
            # captured — not "now". Without this, as the robot drives forward,
            # the TF lookup returns a slightly advanced robot position each frame,
            # making the computed box centre drift backward in odom space.
            #
            # Fallback to latest transform if timestamp lookup fails (e.g. old bag).
            # ----------------------------------------------------------------
            tf_stamped = None
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    'odom',
                    pose_msg.header.frame_id,
                    pose_msg.header.stamp,          # ← image capture time
                    rclpy.duration.Duration(seconds=0.1)
                )
            except (LookupException, ConnectivityException, ExtrapolationException):
                try:
                    tf_stamped = self.tf_buffer.lookup_transform(
                        'odom',
                        pose_msg.header.frame_id,
                        rclpy.time.Time(),          # ← fallback: latest available
                        rclpy.duration.Duration(seconds=0.1)
                    )
                except (LookupException, ConnectivityException, ExtrapolationException) as e:
                    self.get_logger().warn(f"TF lookup failed entirely: {e}")
                    return

            t = tf_stamped.transform.translation
            r = tf_stamped.transform.rotation
            t_map_cam = tft.concatenate_matrices(
                tft.translation_matrix([t.x, t.y, t.z]), 
                tft.quaternion_matrix([r.x, r.y, r.z, r.w])
            )

            # 3. Odom -> Box
            t_box_marker = self.marker_transforms.get(m_id, np.eye(4))
            t_marker_box = tft.inverse_matrix(t_box_marker)
            t_map_box = tft.concatenate_matrices(t_map_cam, t_cam_marker, t_marker_box)

            rot_180 = tft.euler_matrix(0, 0, math.pi) 
            t_map_box = tft.concatenate_matrices(t_map_box, rot_180)

            # 4. Extract Box Pose
            curr_x = t_map_box[0, 3]
            curr_y = t_map_box[1, 3]
            _, _, curr_yaw = tft.euler_from_matrix(t_map_box)

            # Raw camera-to-marker distance — immune to TF jitter
            dist_cam_to_marker = math.sqrt(
                pose.position.x**2 + 
                pose.position.y**2 + 
                pose.position.z**2
            )

            # --- 1HZ DEBUG LOG PER MARKER ---
            now_ns = self.get_clock().now().nanoseconds
            if (now_ns - self.last_log_time.get(m_id, 0)) > 1e9:
                self.last_log_time[m_id] = now_ns
                self.get_logger().info(
                    f"[M{m_id}] "
                    f"raw_dist={dist_cam_to_marker:.3f}m | "
                    f"box=({curr_x:.3f}, {curr_y:.3f}) | "
                    f"yaw={math.degrees(curr_yaw):.1f}° | "
                    f"precision_mode={self.precision_mode} | "
                    f"docking_locked={self.docking_locked} | "
                    f"buffer={len(self.precision_buffer)}/20"
                )

            # --- PRECISION STATE MACHINE ---
            if m_id == 3 and dist_cam_to_marker <= 0.7 and not self.precision_mode:
                self.precision_mode = True
                self.APPROACH_DIST = 0.15
                self.get_logger().info(
                    f"🎯 Entering Precision Mode! M3 raw_dist={dist_cam_to_marker:.3f}m"
                )
                speed_msg = SpeedLimit()
                speed_msg.percentage = False
                speed_msg.speed_limit = 0.15
                self.speed_limit_pub.publish(speed_msg)

            if self.precision_mode:
                if m_id != 3:
                    return

                self.precision_buffer.append((curr_x, curr_y, curr_yaw))

                # Keep publishing mask/goal while buffering so Nav2 keeps moving
                interim_goal_x = curr_x + self.APPROACH_DIST * math.cos(curr_yaw)
                interim_goal_y = curr_y + self.APPROACH_DIST * math.sin(curr_yaw)
                interim_goal_yaw = curr_yaw + math.pi
                self.execute_dynamic_goal(
                    interim_goal_x, interim_goal_y, interim_goal_yaw,
                    curr_x, curr_y, t.x, t.y, final_lock=False
                )

                # Log buffer progress at 1Hz
                if (now_ns - self.last_log_time.get('buffer', 0)) > 1e9:
                    self.last_log_time['buffer'] = now_ns
                    self.get_logger().info(
                        f"📦 Precision buffer: {len(self.precision_buffer)}/20 | "
                        f"M3 dist={dist_cam_to_marker:.3f}m"
                    )

                if len(self.precision_buffer) < 20:
                    return

                # 20 frames collected — lock pristine coordinate
                self.get_logger().info("🔒 20 Frames Collected. Locking pristine coordinate.")
                
                # Strip top/bottom 2 outliers — use distinct loop var 'entry' not 'p'
                x_vals = sorted([entry[0] for entry in self.precision_buffer])
                y_vals = sorted([entry[1] for entry in self.precision_buffer])
                clean_x = sum(x_vals[2:-2]) / 16.0
                clean_y = sum(y_vals[2:-2]) / 16.0

                # Circular mean for yaw
                sin_sum = sum(math.sin(entry[2]) for entry in self.precision_buffer[2:-2])
                cos_sum = sum(math.cos(entry[2]) for entry in self.precision_buffer[2:-2])
                clean_yaw = math.atan2(sin_sum, cos_sum)

                self.docking_locked = True

                speed_msg = SpeedLimit()
                speed_msg.percentage = False
                speed_msg.speed_limit = 0.0
                self.speed_limit_pub.publish(speed_msg)
                
                goal_x = clean_x + self.APPROACH_DIST * math.cos(clean_yaw)
                goal_y = clean_y + self.APPROACH_DIST * math.sin(clean_yaw)
                goal_yaw = clean_yaw + math.pi 

                self.get_logger().info(
                    f"🔒 Final locked goal: box=({clean_x:.3f}, {clean_y:.3f}) | "
                    f"goal=({goal_x:.3f}, {goal_y:.3f}) | "
                    f"yaw={math.degrees(goal_yaw):.1f}°"
                )

                self.publish_box_pose(clean_x, clean_y, clean_yaw)
                self.publish_debug_axes(clean_x, clean_y, clean_yaw)
                
                self.shrink_footprint()
                self.footprint_shrunk = True
                self.execute_dynamic_goal(
                    goal_x, goal_y, goal_yaw,
                    clean_x, clean_y, t.x, t.y,
                    final_lock=True
                )
                return

            # 5. Filtering (rough approach)
            current_time = self.get_clock().now().nanoseconds
            if (current_time - self.last_detection_time.get(m_id, 0)) > 1e9: 
                if m_id in self.pose_history:
                    self.pose_history[m_id].clear()
            self.last_detection_time[m_id] = current_time

            if m_id not in self.pose_history:
                self.pose_history[m_id] = deque(maxlen=self.filter_size)
            self.pose_history[m_id].append((curr_x, curr_y, curr_yaw))

            avg_x = sum(entry[0] for entry in self.pose_history[m_id]) / len(self.pose_history[m_id])
            avg_y = sum(entry[1] for entry in self.pose_history[m_id]) / len(self.pose_history[m_id])
            
            # Circular mean to prevent 180-degree yaw wrap
            sin_sum = sum(math.sin(entry[2]) for entry in self.pose_history[m_id])
            cos_sum = sum(math.cos(entry[2]) for entry in self.pose_history[m_id])
            avg_yaw = math.atan2(sin_sum, cos_sum)

            self.publish_box_pose(avg_x, avg_y, avg_yaw)
            self.publish_debug_axes(avg_x, avg_y, avg_yaw)

            goal_x = avg_x + self.APPROACH_DIST * math.cos(avg_yaw)
            goal_y = avg_y + self.APPROACH_DIST * math.sin(avg_yaw)
            goal_yaw = avg_yaw + math.pi 

            if (now_ns - self.last_log_time.get('goal', 0)) > 1e9:
                self.last_log_time['goal'] = now_ns
                self.get_logger().info(
                    f"📍 Rough approach goal: ({goal_x:.3f}, {goal_y:.3f}) | "
                    f"box=({avg_x:.3f}, {avg_y:.3f}) | "
                    f"yaw={math.degrees(goal_yaw):.1f}°"
                )

            self.execute_dynamic_goal(goal_x, goal_y, goal_yaw, avg_x, avg_y, t.x, t.y, final_lock=False)

        except Exception as e:
            self.get_logger().warn(f"Math Error in solve_box_pose: {e}")

    def publish_box_pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = x
        msg.pose.position.y = y
        q = tft.quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.box_pub.publish(msg)

    def publish_debug_axes(self, x, y, yaw):
        marker_array = MarkerArray()
        
        m1 = Marker()
        m1.header.frame_id = "odom"
        m1.header.stamp = self.get_clock().now().to_msg()
        m1.id = 0
        m1.type = Marker.ARROW
        m1.action = Marker.ADD
        m1.pose.position.x = x
        m1.pose.position.y = y
        m1.pose.position.z = 0.0
        q = tft.quaternion_from_euler(0, 0, yaw)
        m1.pose.orientation.x = q[0]
        m1.pose.orientation.y = q[1]
        m1.pose.orientation.z = q[2]
        m1.pose.orientation.w = q[3]
        m1.scale.x = 0.5; m1.scale.y = 0.05; m1.scale.z = 0.05
        m1.color.r = 1.0; m1.color.a = 1.0
        marker_array.markers.append(m1)

        m2 = Marker()
        m2.header.frame_id = "odom"
        m2.header.stamp = self.get_clock().now().to_msg()
        m2.id = 1
        m2.type = Marker.ARROW
        m2.action = Marker.ADD
        m2.pose.position.x = x
        m2.pose.position.y = y
        m2.pose.position.z = 0.0
        q2 = tft.quaternion_from_euler(0, 0, yaw + math.pi/2)
        m2.pose.orientation.x = q2[0]
        m2.pose.orientation.y = q2[1]
        m2.pose.orientation.z = q2[2]
        m2.pose.orientation.w = q2[3]
        m2.scale.x = 0.3; m2.scale.y = 0.05; m2.scale.z = 0.05
        m2.color.g = 1.0; m2.color.a = 1.0
        marker_array.markers.append(m2)

        self.viz_pub.publish(marker_array)
    
    def shrink_footprint(self):
        poly = Polygon()
        tiny_points = [[-0.04, -0.04], [-0.04, 0.04], [0.04, 0.04], [0.04, -0.04]]
        for pt_coords in tiny_points:
            pt = Point32()
            pt.x, pt.y, pt.z = float(pt_coords[0]), float(pt_coords[1]), 0.0
            poly.points.append(pt)
        self.local_footprint_pub.publish(poly)
        self.global_footprint_pub.publish(poly)
        self.get_logger().info("🛡️ Footprint shrunk! Authorizing entry into the box.")

    def execute_dynamic_goal(self, goal_x, goal_y, goal_yaw, box_x, box_y, robot_x, robot_y, final_lock=False):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "odom"
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        q = tft.quaternion_from_euler(0, 0, goal_yaw)
        goal_msg.pose.orientation.x = q[0]
        goal_msg.pose.orientation.y = q[1]
        goal_msg.pose.orientation.z = q[2]
        goal_msg.pose.orientation.w = q[3]

        # Mask around BOX CENTRE — not the approach point
        mask_msg = PoseStamped()
        mask_msg.header.stamp = self.get_clock().now().to_msg()
        mask_msg.header.frame_id = "odom"
        mask_msg.pose.position.x = box_x
        mask_msg.pose.position.y = box_y
        mask_msg.pose.orientation = goal_msg.pose.orientation
        self.mask_pub.publish(mask_msg)

        distance_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)

        if not self.navigation_started:
            # FIX 2: Non-blocking Nav2 check — was wait_for_server(5.0) which
            # blocked the entire node for 5 seconds every attempt, preventing
            # precision mode from triggering and dropping to 0.2Hz updates
            now_ns = self.get_clock().now().nanoseconds
            if (now_ns - self.last_nav2_attempt_ns) > 5e9:
                self.last_nav2_attempt_ns = now_ns
                if self.send_nav2_goal(goal_msg):
                    self.navigation_started = True
                    self.get_logger().info(
                        f"🚀 Initial goal sent: ({goal_x:.3f}, {goal_y:.3f}) | "
                        f"dist_to_goal={distance_to_goal:.3f}m"
                    )
        else:
            if final_lock or distance_to_goal > 0.5:
                self.update_pub.publish(goal_msg)
    
    def send_nav2_goal(self, pose_stamped):
        self.get_logger().info("🚀 SENDING GOAL TO NAV2...")
        
        # FIX 2 (continued): server_is_ready() is non-blocking — instant check
        # The old wait_for_server(timeout_sec=5.0) blocked Python's GIL for 5s,
        # freezing all callbacks including marker detection
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("Nav2 not available yet, will retry in 5s.")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        self.nav_client.send_goal_async(goal_msg)
        return True

def main():
    rclpy.init()
    rclpy.spin(BoxEstimator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
