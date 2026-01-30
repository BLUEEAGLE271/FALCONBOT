#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations as tft
import numpy as np
import math
from collections import deque

class BoxEstimator(Node):
    def __init__(self):
        super().__init__('box_estimator')

        # --- CONFIGURATION ---
        self.BOX_LENGTH = 0.204   
        self.BOX_WIDTH  = 0.20   
        self.half_L = self.BOX_LENGTH / 2.0
        self.half_W = self.BOX_WIDTH / 2.0
        self.APPROACH_DIST = 0.10 # Distance to stop in front of the box
        
        # Which markers are we looking for?
        self.target_ids = [0, 1, 2, 3]

        # --- STATE ---
        self.navigation_started = False 

        # --- GEOMETRY ---
        self.marker_transforms = {}
        self.setup_box_geometry()

        # --- FILTERING ---
        self.filter_size = 5 
        self.pose_history = {} # Stores deques of (x, y, yaw)

        # --- ROS SETUP ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.update_pub = self.create_publisher(PoseStamped, '/goal_update', 10)
        self.mask_pub = self.create_publisher(PoseStamped, '/parking_goal', 10)
        self.box_pub = self.create_publisher(PoseStamped, '/box_center_pose', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # NEW: Instead of a subscriber, we use a Timer to query TF
        self.timer = self.create_timer(0.1, self.control_loop) # 10 Hz
        
        self.get_logger().info("Box Estimator Ready [TF LISTENER MODE]")

    def setup_box_geometry(self):
        def make_mat_from_vectors(pos, x_vec, y_vec, z_vec):
            mat = np.eye(4)
            mat[0:3, 0] = x_vec
            mat[0:3, 1] = y_vec
            mat[0:3, 2] = z_vec
            mat[0:3, 3] = pos
            return mat

        # Define where markers are relative to the Box Center
        self.marker_transforms[2] = make_mat_from_vectors(
            [self.half_L, 0.0, 0.2], [0, 1, 0], [0, 0, 1], [1, 0, 0]
        )
        self.marker_transforms[0] = make_mat_from_vectors(
            [0.0, -self.half_W, 0.2], [1, 0, 0], [0, 0, 1], [0, -1, 0]
        )
        self.marker_transforms[1] = make_mat_from_vectors(
            [0.0, self.half_W, 0.2], [-1, 0, 0], [0, 0, 1], [0, 1, 0]
        )
        self.marker_transforms[3] = make_mat_from_vectors(
            [-self.half_L, 0.0, 0.2], [0, -1, 0], [0, 0, 1], [-1, 0, 0]
        )

    def control_loop(self):
        # Check for any of the target markers
        for m_id in self.target_ids:
            try:
                # ASK TF: Where is 'marker_X' relative to 'map'?
                # The C++ node publishes 'camera'->'marker_X'
                # The Robot publishes 'map'->'camera'
                # TF2 handles the chain automatically!
                target_frame = f"marker_{m_id}"
                
                # Use Time(0) to get the latest available transform
                trans = self.tf_buffer.lookup_transform(
                    'map', target_frame, rclpy.time.Time()
                )
                
                # Check if data is fresh (within 0.5s)
                now = self.get_clock().now().nanoseconds / 1e9
                tf_time = trans.header.stamp.sec + trans.header.stamp.nanosec / 1e9
                if (now - tf_time) > 0.5:
                    continue # Skip old data

                self.process_marker(m_id, trans)

            except (LookupException, ConnectivityException, ExtrapolationException):
                continue

    def process_marker(self, m_id, tf_stamped):
        # 1. Convert TF message to Matrix
        t = tf_stamped.transform.translation
        r = tf_stamped.transform.rotation
        t_map_marker = tft.concatenate_matrices(
            tft.translation_matrix([t.x, t.y, t.z]), 
            tft.quaternion_matrix([r.x, r.y, r.z, r.w])
        )

        # 2. Apply Geometry Math (Find Box Center from Marker)
        t_box_marker = self.marker_transforms[m_id]
        t_marker_box = tft.inverse_matrix(t_box_marker)
        
        # Box Pose in Map = Marker in Map * Marker to Box
        t_map_box = tft.concatenate_matrices(t_map_marker, t_marker_box)

        # 3. Extract Coordinates
        curr_x = t_map_box[0, 3]
        curr_y = t_map_box[1, 3]
        _, _, curr_yaw = tft.euler_from_matrix(t_map_box)

        # 4. Filtering (Smooth the jitter)
        if m_id not in self.pose_history:
            self.pose_history[m_id] = deque(maxlen=self.filter_size)
        
        self.pose_history[m_id].append((curr_x, curr_y, curr_yaw))

        avg_x = sum(p[0] for p in self.pose_history[m_id]) / len(self.pose_history[m_id])
        avg_y = sum(p[1] for p in self.pose_history[m_id]) / len(self.pose_history[m_id])
        # Simple yaw average (adequate for small jitters)
        avg_yaw = sum(p[2] for p in self.pose_history[m_id]) / len(self.pose_history[m_id])

        # 5. Calculate Parking Goal (Approach Distance offset)
        goal_x = avg_x + self.APPROACH_DIST * math.cos(avg_yaw)
        goal_y = avg_y + self.APPROACH_DIST * math.sin(avg_yaw)
        goal_yaw = avg_yaw + math.pi # Face the box

        # 6. Execute
        self.publish_visualization(avg_x, avg_y, avg_yaw)
        self.execute_dynamic_goal(goal_x, goal_y, goal_yaw)

    def execute_dynamic_goal(self, x, y, yaw):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        q = tft.quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.orientation.x = q[0]
        goal_msg.pose.orientation.y = q[1]
        goal_msg.pose.orientation.z = q[2]
        goal_msg.pose.orientation.w = q[3]

        if not self.navigation_started:
            self.get_logger().info("Target Locked! Starting Nav2...")
            if not self.nav_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn("Nav2 Action Server missing!")
                return

            goal_action = NavigateToPose.Goal()
            goal_action.pose = goal_msg
            self.nav_client.send_goal_async(goal_action)
            self.navigation_started = True
        else:
            # Stream updates to the custom BT node
            self.update_pub.publish(goal_msg)

    def publish_visualization(self, x, y, yaw):
        # Publish where we think the box center is
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        q = tft.quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation.w = q[3] # (Simplified for vis)
        self.mask_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(BoxEstimator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()