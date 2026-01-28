#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient # <--- ADDED
from nav2_msgs.action import NavigateToPose # <--- ADDED
from geometry_msgs.msg import PoseStamped
from aruco_opencv_msgs.msg import ArucoDetection
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft
import numpy as np
import math
from collections import deque # Added for the filter buffer

class BoxEstimator(Node):
    def __init__(self):
        super().__init__('box_estimator')

        # --- CONFIGURATION ---
        self.BOX_LENGTH = 0.204   
        self.BOX_WIDTH  = 0.20   
        self.half_L = self.BOX_LENGTH / 2.0
        self.half_W = self.BOX_WIDTH / 2.0
        self.APPROACH_DIST = 0.10 # Increased slightly for safety during testing

        # --- STATE ---
        self.navigation_started = False # <--- Flag to track if we kicked off Nav2

        # --- GEOMETRY ---
        self.marker_transforms = {}
        self.setup_box_geometry()

        # --- FILTERING ---
        self.filter_size = 5 
        self.pose_history = {} # Stores deques of (x, y, yaw)

        # --- ROS SETUP ---
        # 1. Action Client to START the behavior tree
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 2. Publisher to UPDATE the goal live (The "Dynamic" part)
        self.update_pub = self.create_publisher(PoseStamped, '/goal_update', 10)
        
        # Debug publishers
        self.box_pub = self.create_publisher(PoseStamped, '/box_center_pose', 10)
        self.raw_pub = self.create_publisher(PoseStamped, '/raw_marker_pose', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(ArucoDetection, '/aruco_detections', self.detection_callback, 10)
        self.get_logger().info("Box Estimator Ready [DYNAMIC MODE]")

    def setup_box_geometry(self):
        # ... (Your existing geometry code remains exactly the same) ...
        def make_mat_from_vectors(pos, x_vec, y_vec, z_vec):
            mat = np.eye(4)
            mat[0:3, 0] = x_vec
            mat[0:3, 1] = y_vec
            mat[0:3, 2] = z_vec
            mat[0:3, 3] = pos
            return mat

        self.marker_transforms[2] = make_mat_from_vectors(
            [self.half_L, 0.0, 0.2],  # Position
            [0, 1, 0],                 # X Vector (Flipped)
            [0, 0, 1],                 # Y Vector (Same Z)
            [1, 0, 0]                  # Z Vector (Flipped)
        )
        self.marker_transforms[0] = make_mat_from_vectors(
            [0.0, -self.half_W, 0.2], [1, 0, 0], [0, 0, 1], [0, -1, 0]
        )
        self.marker_transforms[1] = make_mat_from_vectors(
            [0.0, self.half_W, 0.2], [-1, 0, 0], [0, 0, 1], [0, 1, 0]
        )

        self.marker_transforms[3] = make_mat_from_vectors(
            [-self.half_L, 0.0, 0.2], # Position on the front face
            [0, -1, 0],               # X points "Left" from the box perspective
            [0, 0, 1],                # Y points "Up"
            [-1, 0, 0]                # Z points "Forward" (Out of the box)
        )

    def detection_callback(self, msg):
        markers = []
        if hasattr(msg, 'markers'): markers = msg.markers
        elif hasattr(msg, 'id'): markers = [msg]

        for marker in markers:
            # Publish raw for visual debug
            raw = PoseStamped()
            raw.header = msg.header
            raw.header.frame_id = "camera_optical_frame"
            raw.pose = marker.pose
            self.raw_pub.publish(raw)

            if marker.marker_id in self.marker_transforms:
                self.solve_box_pose(marker)

    def solve_box_pose(self, marker_msg):
        try:
            # ... (Your existing math code remains exactly the same) ...
            p = marker_msg.pose
            t_cam_marker = tft.concatenate_matrices(
                tft.translation_matrix([p.position.x, p.position.y, p.position.z]), 
                tft.quaternion_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
            )

            #tf_stamped = self.tf_buffer.lookup_transform('map', 'camera_optical_frame', rclpy.time.Time())
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    'map', 'camera_optical_frame', rclpy.time.Time()
                )
            except (LookupException, ConnectivityException, ExtrapolationException):
                # If TF is not ready, just skip this frame silently
                return
            t = tf_stamped.transform.translation
            r = tf_stamped.transform.rotation
            t_map_cam = tft.concatenate_matrices(
                tft.translation_matrix([t.x, t.y, t.z]), 
                tft.quaternion_matrix([r.x, r.y, r.z, r.w])
            )

            t_box_marker = self.marker_transforms[marker_msg.marker_id]
            t_marker_box = tft.inverse_matrix(t_box_marker)
            t_map_box = tft.concatenate_matrices(t_map_cam, t_cam_marker, t_marker_box)

            rot_180 = tft.euler_matrix(0, 0, math.pi)
            t_map_box = tft.concatenate_matrices(t_map_box, rot_180)

            curr_x = t_map_box[0, 3]
            curr_y = t_map_box[1, 3]
            _, _, curr_yaw = tft.euler_from_matrix(t_map_box)

            # 4. Goal Calculation
            m_id = marker_msg.marker_id

            if m_id == 3:
                current_filter_size = 15 # Much heavier smoothing for the small marker

            
            if m_id not in self.pose_history:
                self.pose_history[m_id] = deque(maxlen=self.filter_size)
            
            self.pose_history[m_id].append((curr_x, curr_y, curr_yaw))

            # Average the values in the buffer
            avg_x = sum(p[0] for p in self.pose_history[m_id]) / len(self.pose_history[m_id])
            avg_y = sum(p[1] for p in self.pose_history[m_id]) / len(self.pose_history[m_id])
            
            # Yaw averaging requires care for wrap-around, but for small jitter, direct avg works:
            avg_yaw = sum(p[2] for p in self.pose_history[m_id]) / len(self.pose_history[m_id])

            # 4. Goal Calculation using Filtered values
            goal_x = avg_x + self.APPROACH_DIST * math.cos(avg_yaw)
            goal_y = avg_y + self.APPROACH_DIST * math.sin(avg_yaw)
            goal_yaw = avg_yaw + math.pi
            goal_dist = self.APPROACH_DIST

            # Back wall fix
           # if marker_msg.marker_id == 2:
            #    goal_x = curr_x + goal_dist * math.cos(curr_yaw)
             #   goal_y = curr_y + goal_dist * math.sin(curr_yaw)
              #  goal_yaw = curr_yaw

            # --- EXECUTION ---
            # Instead of just publishing to a topic, we now choose between
            # STARTING the mission or UPDATING the mission.
            self.execute_dynamic_goal(goal_x, goal_y, goal_yaw)

            # Debug Pub
            ##self.publish_debug_pose(curr_x, curr_y, curr_yaw, self.box_pub)
            self.publish_debug_pose(avg_x, avg_y, avg_yaw, self.box_pub)

        except Exception as e:
            self.get_logger().warn(f"Error: {e}")

    def execute_dynamic_goal(self, x, y, yaw):
        """
        Handles the logic: 
        - If first time: Send Action Goal (Start Behavior Tree)
        - If already running: Publish Update (Stream to GoalUpdater)
        """
        
        # Create the Pose Message
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        msg_q = tft.quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.orientation.x = msg_q[0]
        goal_msg.pose.orientation.y = msg_q[1]
        goal_msg.pose.orientation.z = msg_q[2]
        goal_msg.pose.orientation.w = msg_q[3]

        if not self.navigation_started:
            # --- PHASE 1: KICKOFF ---
            self.get_logger().info("First detection! Starting Dynamic Navigation...")
            
            # Wait for Nav2 to be ready
            if not self.nav_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn("Nav2 Action Server not available yet!")
                return

            # Construct the Action Goal
            goal_action = NavigateToPose.Goal()
            goal_action.pose = goal_msg
            
            # Send it
            self.nav_client.send_goal_async(goal_action)
            
            # Set flag so we don't send action again
            self.navigation_started = True
            
        else:
            # --- PHASE 2: STREAMING ---
            # Just publish the new coordinates to the topic
            self.update_pub.publish(goal_msg)
            # self.get_logger().info(f"Updating goal: {x:.2f}, {y:.2f}", throttle_duration_sec=1.0)

    def publish_debug_pose(self, x, y, yaw, publisher):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        q = tft.quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        publisher.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(BoxEstimator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()