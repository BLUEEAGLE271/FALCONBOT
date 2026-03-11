#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import csv
import threading
import sys
import time
import os
import termios
import tty
import select

class DataCollector(Node):
    def __init__(self):
        super().__init__('pwm_data_collector')
        
        # ROS Publishers and Subscribers
        self.serial_pub = self.create_publisher(String, '/esp32_write', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom_rf2o', self.odom_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        # Velocity Storage
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.lin_data = []
        self.ang_data = []

        # 1. Generate Custom PWM Combinations (Positive Only)
        # 0 to 120 in steps of 40, then 130 to 250 in steps of 10
        pwm_values = list(range(0, 121, 40)) + list(range(130, 260, 10))
        all_combinations = [(l, r) for l in pwm_values for r in pwm_values]

        # 2. Resume Logic: Check existing CSV
        self.csv_filename = 'pwm_velocity_data.csv'
        file_exists = os.path.isfile(self.csv_filename)
        completed_combos = set()

        if file_exists:
            with open(self.csv_filename, 'r') as f:
                reader = csv.reader(f)
                next(reader, None) # Skip header
                for row in reader:
                    if len(row) >= 2:
                        completed_combos.add((int(row[0]), int(row[1])))

        # Filter out already tested combinations
        self.pwm_combinations = [c for c in all_combinations if c not in completed_combos]
        self.total_combos = len(self.pwm_combinations)
        self.current_idx = 0

        # 3. Open CSV in Append Mode
        self.csv_file = open(self.csv_filename, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if not file_exists:
            self.csv_writer.writerow(['pwm_left', 'pwm_right', 'linear_vel', 'angular_vel'])
            self.csv_file.flush()

        if self.total_combos == 0:
            self.get_logger().info("All combinations already tested! File is complete.")
            sys.exit(0)

        self.get_logger().info(f"Resuming... {len(completed_combos)} done, {self.total_combos} remaining.")

        # State Machine Variables
        self.state = 'START_FWD'
        self.start_time = time.time()
        self.is_paused = False

        # Start instant-kill background thread
        threading.Thread(target=self.terminal_listener, daemon=True).start()
        self.get_logger().info("Data Collector Started. Press 'p' to pause, 'r' to resume.")

        # Main Loop Timer (runs at 20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_cb(self, msg):
        self.current_linear = msg.twist.twist.linear.x

    def imu_cb(self, msg):
        self.current_angular = msg.angular_velocity.z

    def send_pwm(self, left, right):
        msg = String()
        msg.data = f'{{"T":11,"L":{left},"R":{right}}}'
        self.serial_pub.publish(msg)

    def terminal_listener(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    char = sys.stdin.read(1).lower()
                    
                    if char == 'p':
                        self.is_paused = True
                        self.send_pwm(0, 0)
                        self.get_logger().info("\n\n>>> [PAUSED] HARDWARE STOPPED. Press 'r' to resume. <<<\n")
                    elif char == 'r':
                        self.is_paused = False
                        self.state = 'START_FWD'
                        self.get_logger().info("\n\n>>> [RESUMED] CONTINUING COLLECTION... <<<\n")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def control_loop(self):
        if self.is_paused or self.current_idx >= self.total_combos:
            if self.current_idx >= self.total_combos:
                self.get_logger().info("Data collection complete!")
                self.csv_file.close()
                rclpy.shutdown()
            return

        current_left, current_right = self.pwm_combinations[self.current_idx]
        now = time.time()
        elapsed = now - self.start_time

        if self.state == 'START_FWD':
            self.lin_data.clear()
            self.ang_data.clear()
            self.send_pwm(current_left, current_right)
            self.start_time = time.time()
            self.state = 'RECORDING_FWD'
            self.get_logger().info(f"[{self.current_idx+1}/{self.total_combos}] Testing L:{current_left} R:{current_right}")

        elif self.state == 'RECORDING_FWD':
            self.lin_data.append(self.current_linear)
            self.ang_data.append(self.current_angular)
            
            if elapsed >= 1.0:
                avg_lin = sum(self.lin_data) / max(1, len(self.lin_data))
                avg_ang = sum(self.ang_data) / max(1, len(self.ang_data))
                
                self.csv_writer.writerow([current_left, current_right, avg_lin, avg_ang])
                self.csv_file.flush() # Force save to disk immediately!
                
                self.send_pwm(0, 0)
                self.start_time = time.time()
                self.state = 'WAIT_SETTLE'

        elif self.state == 'WAIT_SETTLE':
            if elapsed >= 0.5:
                # Return sequence: Apply same power in reverse to get back to start
                self.send_pwm(-current_left, -current_right)
                self.start_time = time.time()
                self.state = 'RETURNING'

        elif self.state == 'RETURNING':
            if elapsed >= 1.0:
                self.send_pwm(0, 0)
                self.start_time = time.time()
                self.state = 'WAIT_END'

        elif self.state == 'WAIT_END':
            if elapsed >= 0.5:
                self.current_idx += 1
                self.state = 'START_FWD'

def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C detected. Shutting down safely...")
        node.send_pwm(0, 0)
        node.csv_file.flush()
        node.csv_file.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()