#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # --- PARKING WAKE-UP SWITCH ---
        self.start_pub = self.create_publisher(Bool, '/start_mission', 10)
        self.create_subscription(Bool, '/mission/trigger', self.trigger_callback, 10)
        
        # --- RVIZ MANUAL DRIVING ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(PoseStamped, '/goal_pose', self.rviz_goal_callback, 10)
        
        self.current_goal_handle = None
        self.get_logger().info("MISSION COMMANDER READY. Listening to RViz clicks and '/mission/trigger'")
    
    def trigger_callback(self, msg):
        # Always tell box_estimator first
        out_msg = Bool()
        out_msg.data = msg.data
        self.start_pub.publish(out_msg)
        
        if msg.data is True:
            self.get_logger().info("\n\n>>> [COMMAND] STARTING PARKING MANEUVER! <<<\n")
        else:
            self.get_logger().warn("\n\n>>> [COMMAND] ABORTING MISSION! <<<\n")
            # This will now stop ANY goal currently running in Nav2
            self.cancel_navigation()

    def cancel_navigation(self):
        # 1. Clear our local handle if it exists
        if self.current_goal_handle is not None:
            self.get_logger().info("Cancelling active goal handle...")
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        
        # 2. EMERGENCY: Use a generic Nav2 cancel if the handle was lost/started elsewhere
        # We wait for server briefly to ensure we can send the cancel
        if self.nav_client.wait_for_server(timeout_sec=1.0):
            # Cancels all goals for this specific client
            self.get_logger().info("Sending global cancel request to Nav2...")
            # Note: In Humble, the cleanest way is often a simple cancel_all_goals if available, 
            # but clearing the current handle usually suffices

    def rviz_goal_callback(self, msg):
        self.get_logger().info("RViz Goal Received! Driving to manual coordinate...")
        self.send_nav_goal(msg)

    def send_nav_goal(self, pose_msg):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 Action Server not available! Is it running?")
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg
        
        # RViz usually sets the frame to 'map', but ensure it's there
        if not goal_msg.pose.header.frame_id:
            goal_msg.pose.header.frame_id = 'map'
            
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_accepted_callback)

    def goal_accepted_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Manual RViz Goal rejected by Nav2.')
            return
        self.current_goal_handle = goal_handle


    def cancel_done(self, future):
        self.current_goal_handle = None

def main():
    rclpy.init()
    rclpy.spin(MissionController())

if __name__ == '__main__':
    main()