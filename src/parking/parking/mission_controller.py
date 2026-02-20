#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool  # <--- FIXED: Added missing import
from nav2_msgs.action import NavigateToPose

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # --- STATE VARIABLES ---
        self.box_found = False
        self.current_goal_handle = None 
        
        # --- AUTO-START CONFIGURATION ---
        # Set this to True so it runs immediately on launch without waiting for button
        self.mission_active = True 
        self.get_logger().info("MISSION: Auto-Start Active! Waiting for exploration goals...")

        # --- ROS HANDLES ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(PoseStamped, '/goal_pose', self.box_callback, 10)
        self.create_subscription(Bool, '/mission/trigger', self.trigger_callback, 10)
        self.create_subscription(PoseStamped, '/exploration_goal', self.explore_callback, 10)

    def trigger_callback(self, msg):
        if msg.data == True:
            self.get_logger().info(">>> MISSION RE-STARTED <<<")
            self.mission_active = True
            self.box_found = False # Reset box state so it explores again
        else:
            self.get_logger().warn(">>> MISSION ABORTED <<<")
            self.cancel_navigation()

    def cancel_navigation(self):
        # 1. Stop the Logic Brain (Ignore new goals)
        self.mission_active = False
        
        # 2. Stop the Nav2 Legs (IMMEDIATELY)
        if self.current_goal_handle is not None:
            self.get_logger().info("Cancelling current Nav2 Goal...")
            future = self.current_goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done)
        else:
            self.get_logger().info("No active goal to cancel.")

    def cancel_done(self, future):
        self.get_logger().info("Robot has effectively stopped.")
        self.current_goal_handle = None

    def box_callback(self, msg):
        if not self.mission_active: return 
        
        if not self.box_found:
            self.get_logger().info("MISSION: BOX DETECTED! Switching to Box Mode.")
            self.box_found = True
            # Optional: Cancel current exploration goal to prioritize box immediately
            if self.current_goal_handle:
                self.current_goal_handle.cancel_goal_async()
        
        self.send_nav_goal(msg, "BOX PARKING")

    def explore_callback(self, msg):
        if not self.mission_active: return 
        if self.box_found: return # Don't explore if we found the box
            
        self.get_logger().info("MISSION: Exploring new frontier...")
        self.send_nav_goal(msg, "EXPLORING")

    def send_nav_goal(self, pose_msg, mode_name):
        self.nav_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg.pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info(f"[{mode_name}] Sending Goal: {pose_msg.pose.position.x:.2f}, {pose_msg.pose.position.y:.2f}")
        
        # --- FIXED: Save the Future to get the Handle ---
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_accepted_callback)

    # --- NEW: Callback to capture the Goal Handle ---
    def goal_accepted_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        # FIXED: We save this handle so cancel_navigation() can use it later
        self.current_goal_handle = goal_handle

def main():
    rclpy.init()
    rclpy.spin(MissionController())

if __name__ == '__main__':
    main()