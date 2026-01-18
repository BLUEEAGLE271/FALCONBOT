#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ScanRepublisher(Node):
    def __init__(self):
        super().__init__('scan_republisher')

        # 1. Define QoS: RELIABLE (To match your LD19 Lidar)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 2. Subscribe using RELIABLE
        self.sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.listener_callback, 
            qos_profile
        )

        # 3. Publish to Ghost Topic (Reliable)
        self.pub = self.create_publisher(LaserScan, '/scan_nav', 10)
        
        self.get_logger().info("Scan Republisher Started. Waiting for data...")
        self.count = 0

    def listener_callback(self, msg):
        # DEBUG: Print once every 40 scans (approx 2 seconds) to prove it works
        self.count += 1
        if self.count % 40 == 0:
            self.get_logger().info(f"Relaying scan {self.count} - Frame: {msg.header.frame_id} -> base_laser_nav")

        # OVERRIDE FRAME
        msg.header.frame_id = 'base_laser_nav'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()