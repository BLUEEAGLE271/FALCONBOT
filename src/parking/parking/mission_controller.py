#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # Publisher to wake up Box Estimator
        self.start_pub = self.create_publisher(Bool, '/start_mission', 10)
        
        # Subscriber for your terminal command
        self.create_subscription(Bool, '/mission/trigger', self.trigger_callback, 10)
        
        self.get_logger().info("MISSION COMMANDER READY. Waiting for True/False on '/mission/trigger'")
    
    def trigger_callback(self, msg):
        out_msg = Bool()
        out_msg.data = msg.data
        
        if msg.data is True:
            self.get_logger().info("\n\n>>> [COMMAND] STARTING PARKING MANEUVER! <<<\n")
        else:
            self.get_logger().warn("\n\n>>> [COMMAND] ABORTING PARKING MANEUVER! <<<\n")
            
        # Immediately forward the command to Box Estimator
        self.start_pub.publish(out_msg)

def main():
    rclpy.init()
    rclpy.spin(MissionController())

if __name__ == '__main__':
    main()
