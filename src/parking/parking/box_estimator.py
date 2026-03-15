#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray # <--- Added for Axes
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations as tft
import numpy as np
import math
from collections import deque
from std_msgs.msg import Bool  # <--- MUST BE HERE
from geometry_msgs.msg import PoseStamped, Polygon, Point32

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
        self.filter_size = 5 
        self.pose_history = {} 

        # --- ROS SETUP ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.update_pub = self.create_publisher(PoseStamped, '/goal_update', 10)
        self.mask_pub = self.create_publisher(PoseStamped, '/parking_goal', 10)
        self.box_pub = self.create_publisher(PoseStamped, '/box_center_pose', 10) # <--- You asked for this
        self.viz_pub = self.create_publisher(MarkerArray, '/box_debug_axes', 10)  # <--- NEW DEBUG VISUALIZER
        self.local_footprint_pub = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.global_footprint_pub = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Dynamic Subscription Mode (Preferred)
        self.subs = []
        for m_id in self.target_ids:
            topic_name = f'/aruco/marker_{m_id}'
            sub = self.create_subscription(
                PoseStamped, 
                topic_name, 
                lambda msg, mid=m_id: self.detection_callback(msg, mid), 
                10
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
            return  # Ignore all markers until mission starts
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

        # --- MARKER 1 IS ON THE LEFT FACE ---
        # Left Face means it is at +Y (relative to box center)
        # Its Normal (Z) points OUT, so it points +Y.
        # Its Up (Y) points UP, so it points +Z.
        # Cross product (Y cross Z) = X.  (0,0,1)x(0,1,0) = (-1,0,0).
        # So Marker X points to Box -X.
        self.marker_transforms[1] = make_mat(
            [0.0, self.half_W, 0.2],  # Pos: Left side (+Y)
            [-1, 0, 0],               # X-Vec: Points Back (-X)
            [0, 0, 1],                # Y-Vec: Points Up (+Z)
            [0, 1, 0]                 # Z-Vec: Points Left (+Y) [Normal]
        )

        # Marker 0 (Right Face - Assumption)
        self.marker_transforms[0] = make_mat(
            [0.0, -self.half_W, 0.2], 
            [1, 0, 0], 
            [0, 0, 1], 
            [0, -1, 0]
        )

        self.marker_transforms[2] = make_mat(
            [-self.half_L+self.half_L+self.APPROACH_DIST, 0.0, 0.2], # Pos: Back side
            [0, 1, 0],                # X-Vec: Points Left (+Y)
            [0, 0, 1],                # Y-Vec: Points Up (+Z)
            [1, 0, 0]                 # Z-Vec: Points Front (+X) - Aligns with Box X
        )

        self.marker_transforms[3] = make_mat(
            [self.half_L-0.06, 0.0, 0.2],  # Position: Front
            [0, -1, 0],               # Marker X aligns with Box -Y
            [0, 0, 1],                # Marker Y aligns with Box Z
            [-1, 0, 0]                 # Marker Z aligns with Box X (Front)
        )


    def solve_box_pose(self, pose_msg, m_id):
        try:
            # 1. Camera -> Marker
            p = pose_msg.pose
            t_cam_marker = tft.concatenate_matrices(
                tft.translation_matrix([p.position.x, p.position.y, p.position.z]), 
                tft.quaternion_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
            )

            # 2. Map -> Camera
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    'odom', pose_msg.header.frame_id, rclpy.time.Time()
                )
            except (LookupException, ConnectivityException, ExtrapolationException):
                return

            t = tf_stamped.transform.translation
            r = tf_stamped.transform.rotation
            t_map_cam = tft.concatenate_matrices(
                tft.translation_matrix([t.x, t.y, t.z]), 
                tft.quaternion_matrix([r.x, r.y, r.z, r.w])
            )

            # 3. Map -> Box
            t_box_marker = self.marker_transforms.get(m_id, np.eye(4))
            t_marker_box = tft.inverse_matrix(t_box_marker)
            t_map_box = tft.concatenate_matrices(t_map_cam, t_cam_marker, t_marker_box)

            rot_180 = tft.euler_matrix(0, 0, math.pi) 
            t_map_box = tft.concatenate_matrices(t_map_box, rot_180)

            # 4. Extract Box Pose
            curr_x = t_map_box[0, 3]
            curr_y = t_map_box[1, 3]
            _, _, curr_yaw = tft.euler_from_matrix(t_map_box)

            # 5. Filtering
            if m_id not in self.pose_history:
                self.pose_history[m_id] = deque(maxlen=self.filter_size)
            self.pose_history[m_id].append((curr_x, curr_y, curr_yaw))

            sin_sum = sum(math.sin(p[2]) for p in self.pose_history[m_id])
            cos_sum = sum(math.cos(p[2]) for p in self.pose_history[m_id])
            avg_yaw = math.atan2(sin_sum, cos_sum)

            # 6. Publish Box Center (For Debugging)
            self.publish_box_pose(avg_x, avg_y, avg_yaw)
            
            # 7. Publish Debug Axes (To see Rotation)
            self.publish_debug_axes(avg_x, avg_y, avg_yaw)

            # 8. Calculate Goal (Into the box)
            # Center + Offset. If Box X is "Front", we want to approach along X.
            safe_offset = self.APPROACH_DIST
            goal_x = avg_x + safe_offset * math.cos(avg_yaw)
            goal_y = avg_y + safe_offset * math.sin(avg_yaw)
            goal_yaw = avg_yaw + math.pi # Face opposite to Box Front

            self.execute_dynamic_goal(goal_x, goal_y, goal_yaw, t.x, t.y)

        except Exception as e:
            self.get_logger().warn(f"Math Error: {e}")

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
        
        # RED ARROW = Box X Axis (Front)
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

        # GREEN ARROW = Box Y Axis (Left)
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
        
        # Define a microscopic footprint (1cm radius)
        
        tiny_points = [[-0.04, -0.04], [-0.04, 0.04], [0.04, 0.04], [0.04, -0.04]]
        for p in tiny_points:
            pt = Point32()
            pt.x, pt.y, pt.z = float(p[0]), float(p[1]), 0.0
            poly.points.append(pt)
            
        # Tell both costmaps to update their collision math immediately
        self.local_footprint_pub.publish(poly)
        self.global_footprint_pub.publish(poly)
        self.get_logger().info("🛡️ Footprint shrunk! Authorizing entry into the box.")

    def execute_dynamic_goal(self, x, y, yaw, robot_x, robot_y):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "odom"
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        q = tft.quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.orientation.x = q[0]
        goal_msg.pose.orientation.y = q[1]
        goal_msg.pose.orientation.z = q[2]
        goal_msg.pose.orientation.w = q[3]
        
        # 1. Always publish to Masking Node (High Frequency is OK here)
        self.mask_pub.publish(goal_msg)

        distance_to_goal = math.hypot(x - robot_x, y - robot_y)

        if distance_to_goal < 0.10:
            # We are close enough. Stop sending updates so MPPI can park smoothly without jitter.
            return
        if distance_to_goal < 0.4 and not self.footprint_shrunk:
            self.get_logger().info(f"Target is {distance_to_goal:.2f}m away! Dropping shields and shrinking footprint.")
            self.shrink_footprint()
            self.footprint_shrunk = True

        
        # 2. Dynamic Nav2 Updates (The GoalUpdater Method)
        if not self.navigation_started:
            # PHASE 1: Wait for a TRUE response before switching phases
            if self.send_nav2_goal(goal_msg):
                self.navigation_started = True
                self.get_logger().info("Initial Action Goal Sent. Switching to dynamic topic updates.")
        else:
            # PHASE 2: Only update the goal if we are further than 0.5m away
            if distance_to_goal > 0.5:
                self.update_pub.publish(goal_msg)
            else:
                # We are closer than 0.5m. Do NOTHING. 
                # This starves the GoalUpdatedController, forcing the robot 
                # to just finish driving the exact path it already has.
                pass
    
    def send_nav2_goal(self, pose_stamped):
        self.get_logger().info("🚀 SENDING GOAL TO NAV2...")
        
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 Action Server not available! Will try again.")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        self.nav_client.send_goal_async(goal_msg)
        return True # Success!

def main():
    rclpy.init()
    rclpy.spin(BoxEstimator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
