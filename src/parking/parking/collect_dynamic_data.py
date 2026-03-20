#!/usr/bin/env python3
"""
Dynamic Training Data Collector
=================================
Drives the robot through random step commands and records
the full transient velocity response for NN training.

Features:
  - Captures full transient from t=0 (not just steady state)
  - SPACE to pause/resume — reposition robot between steps
  - 'q' to quit and save
  - Appends to existing CSV so you can collect in multiple sessions
  - Progress saved after every step — no data lost on Ctrl+C

Output: ~/ros2_ws/training_data/dynamic_data.csv

Usage:
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  python3 collect_dynamic_data.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import threading
import time
import csv
import os
import random
import sys
import termios
import tty
import select


# ── Tunable parameters ────────────────────────────────────────────────
PWM_STEPS  = [40, 80, 120, 160, 200, 240]  # PWM magnitudes to test
                                             # Start at 40 — no zeros, always moving
HOLD_TIME  = 1.2    # seconds to hold each PWM step — captures full transient
SETTLE_TIME = 0.8   # seconds at zero between steps — robot fully stops
SAMPLE_HZ  = 25     # samples per second during hold
NUM_COMBOS = 200    # total random combinations to run
ODOM_TOPIC = '/odom'
PWM_TOPIC  = '/esp32_write'
# ─────────────────────────────────────────────────────────────────────


class KeyReader:
    """Non-blocking single character reader from terminal."""
    def __init__(self):
        self._fd       = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)

    def __enter__(self):
        tty.setraw(self._fd)
        return self

    def __exit__(self, *args):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def key_available(self):
        return select.select([sys.stdin], [], [], 0)[0] != []

    def read_key(self):
        return sys.stdin.read(1)


class DynamicDataCollector(Node):
    def __init__(self):
        super().__init__('dynamic_data_collector')

        be_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._lock    = threading.Lock()
        self._vx      = 0.0
        self._wz      = 0.0
        self._odom_ok = False

        self.odom_sub  = self.create_subscription(
            Odometry, ODOM_TOPIC, self._odom_cb, be_qos)
        self.serial_pub = self.create_publisher(String, PWM_TOPIC, 10)

    def _odom_cb(self, msg):
        with self._lock:
            self._vx      = msg.twist.twist.linear.x
            self._wz      = msg.twist.twist.angular.z
            self._odom_ok = True

    def get_velocity(self):
        with self._lock:
            return self._vx, self._wz

    def send_pwm(self, left, right):
        msg      = String()
        msg.data = f'{{"T":11,"L":{left:.0f},"R":{right:.0f}}}'
        self.serial_pub.publish(msg)

    def stop(self):
        self.send_pwm(0, 0)


def generate_combos(n):
    """
    Generate diverse (left, right) PWM combinations.
    Covers: straight, curved, in-place rotation, mixed.
    All combinations guaranteed to produce motion (no double-zero).
    """
    combos = []

    # Straight forward/backward
    for pwm in PWM_STEPS:
        combos.append(( pwm,  pwm))
        combos.append((-pwm, -pwm))

    # In-place rotation
    for pwm in PWM_STEPS:
        combos.append(( pwm, -pwm))
        combos.append((-pwm,  pwm))

    # Gentle curves (one wheel dominant)
    for pwm in PWM_STEPS:
        for slow in [0, pwm//2]:
            combos.append((pwm,  slow))
            combos.append((slow, pwm))
            combos.append((-pwm, -slow))
            combos.append((-slow, -pwm))

    # Random asymmetric
    signs = [1, -1]
    for _ in range(n - len(combos)):
        l = random.choice(PWM_STEPS) * random.choice(signs)
        r = random.choice(PWM_STEPS) * random.choice(signs)
        combos.append((l, r))

    random.shuffle(combos)
    return combos[:n]


def collect_data():
    rclpy.init()
    node = DynamicDataCollector()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Wait for odom
    print("Waiting for /odom...")
    while not node._odom_ok:
        time.sleep(0.1)
    print("✅ Odom ready.\n")

    # Output setup — append to existing file
    out_dir  = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'training_data')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'dynamic_data.csv')

    fieldnames = ['pwm_left', 'pwm_right',
                  'vx_before', 'wz_before',
                  'vx_after', 'wz_after', 'dt']

    file_exists = os.path.exists(out_path)
    csv_file = open(out_path, 'a', newline='')
    writer   = csv.DictWriter(csv_file, fieldnames=fieldnames)
    if not file_exists:
        writer.writeheader()
        csv_file.flush()

    combos      = generate_combos(NUM_COMBOS)
    total_rows  = 0
    paused      = False
    quit_flag   = False

    print("="*60)
    print("  DYNAMIC DATA COLLECTOR")
    print(f"  Running {len(combos)} combinations")
    print(f"  Hold={HOLD_TIME}s  Settle={SETTLE_TIME}s  Sample={SAMPLE_HZ}Hz")
    print("")
    print("  Controls (press key in this terminal):")
    print("    SPACE — pause/resume (stop robot, reposition it)")
    print("    q     — quit and save")
    print("="*60 + "\n")

    dt = 1.0 / SAMPLE_HZ

    with KeyReader() as kr:
        for i, (pwm_l, pwm_r) in enumerate(combos):

            # ── Check keyboard ────────────────────────────────────
            if kr.key_available():
                key = kr.read_key()
                if key == ' ':
                    paused = not paused
                    if paused:
                        node.stop()
                        print(f"\n⏸️  PAUSED after combo {i}/{len(combos)} "
                              f"| {total_rows} rows saved")
                        print("   Reposition robot, then press SPACE to resume.")
                    else:
                        print(f"\n▶️  RESUMED — combo {i}/{len(combos)}")
                elif key in ('q', 'Q'):
                    quit_flag = True

            if quit_flag:
                break

            # ── Wait while paused ─────────────────────────────────
            while paused:
                time.sleep(0.1)
                if kr.key_available():
                    key = kr.read_key()
                    if key == ' ':
                        paused = False
                        print(f"\n▶️  RESUMED — combo {i}/{len(combos)}")
                    elif key in ('q', 'Q'):
                        quit_flag = True
                        paused    = False
                if quit_flag:
                    break

            if quit_flag:
                break

            # ── 1. Settle at zero ─────────────────────────────────
            node.stop()
            time.sleep(SETTLE_TIME)

            # Record velocity just before the step
            vx_before, wz_before = node.get_velocity()

            # ── 2. Apply PWM step + sample full transient ─────────
            node.send_pwm(pwm_l, pwm_r)
            t_start   = time.time()
            step_rows = 0

            while (time.time() - t_start) < HOLD_TIME:
                time.sleep(dt)
                elapsed           = time.time() - t_start
                vx_after, wz_after = node.get_velocity()

                writer.writerow({
                    'pwm_left':  pwm_l,
                    'pwm_right': pwm_r,
                    'vx_before': round(vx_before, 5),
                    'wz_before': round(wz_before, 5),
                    'vx_after':  round(vx_after,  5),
                    'wz_after':  round(wz_after,  5),
                    'dt':        round(elapsed,    4)
                })

                # Update vx_before for next sample (rolling window)
                vx_before = vx_after
                wz_before = wz_after
                step_rows += 1

            csv_file.flush()  # save to disk after every step
            total_rows += step_rows

            # Progress line
            vx_ss, wz_ss = node.get_velocity()
            print(f"  [{i+1:3d}/{len(combos)}] "
                  f"L={pwm_l:4d} R={pwm_r:4d} | "
                  f"vx_ss={vx_ss:+.3f} wz_ss={wz_ss:+.3f} | "
                  f"+{step_rows} rows | total={total_rows}  "
                  f"[SPACE=pause  q=quit]",
                  end='\r')

    # ── Cleanup ───────────────────────────────────────────────────────
    node.stop()
    csv_file.close()

    print(f"\n\n✅ Collection complete.")
    print(f"   Total rows: {total_rows}")
    print(f"   Saved to:   {out_path}")

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    collect_data()