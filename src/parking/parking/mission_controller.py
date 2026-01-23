#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        self.box_found = False
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 1. Listen for BOX Goals (High Priority) on default topic
        self.create_subscription(PoseStamped, '/goal_pose', self.box_callback, 10)

        # 2. Listen for EXPLORATION Goals (Low Priority) - We will remap the explorer to this
        self.create_subscription(PoseStamped, '/exploration_goal', self.explore_callback, 10)
        
        self.get_logger().info("MISSION: Ready. Waiting for exploration or box goals.")

    def box_callback(self, msg):
        # Once we find the box, we ignore exploration forever
        if not self.box_found:
            self.get_logger().info("MISSION: BOX DETECTED! Switching to Box Mode.")
            self.box_found = True
        
        # Always execute box goals (updates parking alignment)
        self.send_nav_goal(msg, "BOX PARKING")

    def explore_callback(self, msg):
        # Only explore if we haven't found the box yet
        if self.box_found:
            return 
            
        self.get_logger().info("MISSION: Exploring new frontier...")
        self.send_nav_goal(msg, "EXPLORING")

    def send_nav_goal(self, pose_msg, mode_name):
        self.nav_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg.pose
        goal_msg.pose.header.frame_id = 'map'
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map" # or "odom"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = msg.pose # Assuming msg.pose is the geometry_msgs/Pose
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f"[{mode_name}] Sending Goal: {pose_msg.pose.position.x:.2f}, {pose_msg.pose.position.y:.2f}")
        
        self.nav_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    rclpy.spin(MissionController())

if __name__ == '__main__':
    main()