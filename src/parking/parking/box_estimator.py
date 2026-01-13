#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from aruco_opencv_msgs.msg import ArucoDetection
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft
import numpy as np
import math

class BoxEstimator(Node):
    def __init__(self):
        super().__init__('box_estimator')

        # --- CONFIGURATION ---
        self.BOX_LENGTH = 0.204   # 23 cm
        self.BOX_WIDTH  = 0.20   # 20 cm
        
        self.half_L = self.BOX_LENGTH / 2.0
        self.half_W = self.BOX_WIDTH / 2.0
        self.APPROACH_DIST = 0.08 

        # --- GEOMETRY ---
        self.marker_transforms = {}
        self.setup_box_geometry()

        # --- ROS SETUP ---
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.box_pub = self.create_publisher(PoseStamped, '/box_center_pose', 10)
        self.raw_pub = self.create_publisher(PoseStamped, '/raw_marker_pose', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(ArucoDetection, '/aruco_detections', self.detection_callback, 10)
        self.get_logger().info("Box Estimator Ready [DEBUG MODE]")


    def setup_box_geometry(self):
        def make_mat_from_vectors(pos, x_vec, y_vec, z_vec):
            mat = np.eye(4)
            mat[0:3, 0] = x_vec
            mat[0:3, 1] = y_vec
            mat[0:3, 2] = z_vec
            mat[0:3, 3] = pos
            return mat

        # ID 2: BACK WALL (FIXED)
        # We align this to be "Upright" like ID 0.
        self.marker_transforms[2] = make_mat_from_vectors(
            [-self.half_L, 0.0, 0.2], 
            [0, -1, 0],  # X (Red) points Box Right (-Y)
            [0, 0, 1],   # Y (Green) points Box Up (+Z)
            [-1, 0, 0]   # Z (Blue) points Box Back (-X)
        )

        # ID 0: RIGHT WALL (Unchanged, assumed correct)
        self.marker_transforms[0] = make_mat_from_vectors(
            [0.0, -self.half_W, 0.2], 
            [1, 0, 0], 
            [0, 0, 1], 
            [0, -1, 0]
        )

        # ID 1: LEFT WALL (Unchanged)
        self.marker_transforms[1] = make_mat_from_vectors(
            [0.0, self.half_W, 0.2], 
            [-1, 0, 0], 
            [0, 0, 1], 
            [0, 1, 0]
        )

        self.marker_transforms[3] = make_mat_from_vectors(
            [-self.half_L, 0.0, 0.2], 
            [0, 1, 0],   # X points Left (+Y)
            [0, 0, 1],   # Y points Up (+Z)
            [1, 0, 0]    # Z points Front (+X)
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
            # 1. Cam -> Marker
            p = marker_msg.pose
            t_cam_marker = tft.concatenate_matrices(
                tft.translation_matrix([p.position.x, p.position.y, p.position.z]), 
                tft.quaternion_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
            )

            # 2. Map -> Camera
            tf_stamped = self.tf_buffer.lookup_transform('map', 'camera_optical_frame', rclpy.time.Time())
            t = tf_stamped.transform.translation
            r = tf_stamped.transform.rotation
            t_map_cam = tft.concatenate_matrices(
                tft.translation_matrix([t.x, t.y, t.z]), 
                tft.quaternion_matrix([r.x, r.y, r.z, r.w])
            )

            # 3. Chain
            t_box_marker = self.marker_transforms[marker_msg.marker_id]
            t_marker_box = tft.inverse_matrix(t_box_marker)
            t_map_box = tft.concatenate_matrices(t_map_cam, t_cam_marker, t_marker_box)

            box_x = t_map_box[0, 3]
            box_y = t_map_box[1, 3]
            _, _, box_yaw = tft.euler_from_matrix(t_map_box)

            # 4. Goal
            goal_dist = self.APPROACH_DIST
            goal_x = box_x + goal_dist * math.cos(box_yaw + math.pi)
            goal_y = box_y + goal_dist * math.sin(box_yaw + math.pi)
            goal_yaw = box_yaw + math.pi
            yaw_deg = math.degrees(box_yaw)
            if marker_msg.marker_id == 2:
                goal_x = box_x + goal_dist * math.cos(box_yaw)
                goal_y = box_y + goal_dist * math.sin(box_yaw)
                goal_yaw = box_yaw
                yaw_deg = math.degrees(box_yaw-math.pi)
            
            
            

            # --- DEEP DEBUGGING ---
            
            raw_z = marker_msg.pose.position.z
            
            face = "UNKNOWN"
            if marker_msg.marker_id == 2: face = "BACK"

            self.get_logger().info(
                f"\n--- DEBUG ID {marker_msg.marker_id} ({face}) ---\n"
                f"1. Raw Z Dist: {raw_z:.3f} m\n"
                f"2. Box Center: X={box_x:.3f}, Y={box_y:.3f}\n"
                f"3. Box Yaw:    {yaw_deg:.1f} deg (Cos: {math.cos(box_yaw):.2f})\n"
                f"3. Goal Yaw:    {yaw_deg:.1f} deg (Cos: {math.cos(goal_yaw):.2f})\n"
                f"5. Final Gal: X={goal_x:.3f}, Y={goal_y:.3f}\n"
                f"-----------------------------"
            )

            self.publish_pose(box_x, box_y, box_yaw, self.box_pub)
            self.publish_pose(goal_x, goal_y, goal_yaw, self.goal_pub)

        except Exception as e:
            self.get_logger().warn(f"Error: {e}")

    def publish_pose(self, x, y, yaw, publisher):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
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