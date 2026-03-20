#!/usr/bin/env python3
"""
Mission Data Recorder & Plotter (with PWM)
===========================================
Records and plots everything during a docking mission:
  - Robot position over time
  - cmd_vel (target velocity) over time
  - Actual velocity (odom) over time
  - Left/Right wheel PWM over time
  - Goal pose updates over time
  - Nav2 planned path — first AND last plan
  - Box centre estimate over time

Usage:
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  python3 mission_recorder.py

Press Ctrl+C to stop recording and generate plots.
Saves CSVs + PNG to /tmp/mission_<timestamp>/
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import threading
import time
import os
import csv
import math
from datetime import datetime


class MissionRecorder(Node):
    def __init__(self):
        super().__init__('mission_recorder')

        be_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._lock = threading.Lock()
        self._t0   = time.time()

        # cmd_vel
        self._t_cmd, self._cmd_vx, self._cmd_wz = [], [], []
        # odom velocity
        self._t_odom, self._odom_vx, self._odom_wz = [], [], []
        # robot position
        self._t_pos, self._pos_x, self._pos_y, self._pos_yaw = [], [], [], []
        # PWM
        self._t_pwm, self._pwm_left, self._pwm_right = [], [], []
        # goal updates
        self._t_goal, self._goal_x, self._goal_y, self._goal_yaw = [], [], [], []
        # box centre
        self._t_box, self._box_x, self._box_y = [], [], []
        # Nav2 plans — keep first and last
        self._plan_first_x, self._plan_first_y = [], []
        self._plan_last_x,  self._plan_last_y  = [], []
        self._plan_count = 0

        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, be_qos)
        self.create_subscription(String, '/motor_pwm', self._pwm_cb, 10)
        self.create_subscription(PoseStamped, '/goal_update', self._goal_cb, 10)
        self.create_subscription(PoseStamped, '/box_center_pose', self._box_cb, 10)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)

        self.get_logger().info("Mission Recorder started. Press Ctrl+C to stop.")

    def _t(self):
        return time.time() - self._t0

    def _cmd_cb(self, msg):
        t = self._t()
        with self._lock:
            self._t_cmd.append(t)
            self._cmd_vx.append(msg.linear.x)
            self._cmd_wz.append(msg.angular.z)

    def _odom_cb(self, msg):
        t = self._t()
        vx  = msg.twist.twist.linear.x
        wz  = msg.twist.twist.angular.z
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        with self._lock:
            self._t_odom.append(t); self._odom_vx.append(vx); self._odom_wz.append(wz)
            self._t_pos.append(t);  self._pos_x.append(x);   self._pos_y.append(y)
            self._pos_yaw.append(yaw)

    def _pwm_cb(self, msg):
        t = self._t()
        try:
            parts = msg.data.split(',')
            l, r = float(parts[0]), float(parts[1])
            with self._lock:
                self._t_pwm.append(t); self._pwm_left.append(l); self._pwm_right.append(r)
        except Exception:
            pass

    def _goal_cb(self, msg):
        t = self._t()
        q   = msg.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        with self._lock:
            self._t_goal.append(t); self._goal_x.append(msg.pose.position.x)
            self._goal_y.append(msg.pose.position.y); self._goal_yaw.append(math.degrees(yaw))

    def _box_cb(self, msg):
        t = self._t()
        with self._lock:
            self._t_box.append(t); self._box_x.append(msg.pose.position.x)
            self._box_y.append(msg.pose.position.y)

    def _plan_cb(self, msg):
        xs = [p.pose.position.x for p in msg.poses]
        ys = [p.pose.position.y for p in msg.poses]
        with self._lock:
            if self._plan_count == 0:
                self._plan_first_x = xs; self._plan_first_y = ys
            self._plan_last_x = xs; self._plan_last_y = ys
            self._plan_count += 1

    def save_and_plot(self, out_dir):
        os.makedirs(out_dir, exist_ok=True)

        with self._lock:
            t_cmd = list(self._t_cmd); cmd_vx = list(self._cmd_vx); cmd_wz = list(self._cmd_wz)
            t_odom = list(self._t_odom); odom_vx = list(self._odom_vx); odom_wz = list(self._odom_wz)
            t_pos = list(self._t_pos); pos_x = list(self._pos_x); pos_y = list(self._pos_y); pos_yaw = list(self._pos_yaw)
            t_pwm = list(self._t_pwm); pwm_left = list(self._pwm_left); pwm_right = list(self._pwm_right)
            t_goal = list(self._t_goal); goal_x = list(self._goal_x); goal_y = list(self._goal_y); goal_yaw = list(self._goal_yaw)
            t_box = list(self._t_box); box_x = list(self._box_x); box_y = list(self._box_y)
            plan_first_x = list(self._plan_first_x); plan_first_y = list(self._plan_first_y)
            plan_last_x = list(self._plan_last_x); plan_last_y = list(self._plan_last_y)
            plan_count = self._plan_count

        def save_csv(fname, headers, *cols):
            if not cols[0]: return
            path = os.path.join(out_dir, fname)
            with open(path, 'w', newline='') as f:
                w = csv.writer(f); w.writerow(headers)
                for row in zip(*cols): w.writerow(row)
            print(f"  saved {path}")

        save_csv('cmd_vel.csv',       ['time_s','cmd_vx','cmd_wz'],              t_cmd, cmd_vx, cmd_wz)
        save_csv('odom_velocity.csv', ['time_s','odom_vx','odom_wz'],            t_odom, odom_vx, odom_wz)
        save_csv('robot_position.csv',['time_s','x','y','yaw_rad'],              t_pos, pos_x, pos_y, pos_yaw)
        save_csv('motor_pwm.csv',     ['time_s','pwm_left','pwm_right'],         t_pwm, pwm_left, pwm_right)
        save_csv('goal_updates.csv',  ['time_s','goal_x','goal_y','yaw_deg'],    t_goal, goal_x, goal_y, goal_yaw)
        save_csv('box_estimate.csv',  ['time_s','box_x','box_y'],                t_box, box_x, box_y)

        print(f"\nSummary:")
        if t_pos: print(f"  Duration: {t_pos[-1]:.1f}s  |  Odom samples: {len(t_odom)}")
        print(f"  Cmd samples: {len(t_cmd)}  |  PWM samples: {len(t_pwm)}")
        print(f"  Goal updates: {len(t_goal)}  |  Nav2 replans: {plan_count}")
        if pwm_left:
            print(f"  PWM left  mean/max: {np.mean(pwm_left):.1f} / {np.max(np.abs(pwm_left)):.1f}")
            print(f"  PWM right mean/max: {np.mean(pwm_right):.1f} / {np.max(np.abs(pwm_right)):.1f}")

        # --- PLOT ---
        BG='#0d1117'; CARD='#161b22'; GRID='#21262d'; AXIS='#8b949e'
        BLUE='#58a6ff'; GREEN='#3fb950'; RED='#f78166'; PURP='#d2a8ff'
        ORG='#ffa657'; TEAL='#39d353'; YELL='#e3b341'; PINK='#ff7eb6'

        has_pwm = len(t_pwm) > 5

        fig = plt.figure(figsize=(20, 16), facecolor=BG)
        fig.suptitle('Mission Recording — Full Analysis',
                     color='white', fontsize=14, fontweight='bold', y=0.99)

        gs = gridspec.GridSpec(4, 3, figure=fig, hspace=0.55, wspace=0.38)
        ax_map = fig.add_subplot(gs[0:2, 0:2])
        ax_gx  = fig.add_subplot(gs[0, 2])
        ax_gy  = fig.add_subplot(gs[1, 2])
        ax_vx  = fig.add_subplot(gs[2, 0])
        ax_wz  = fig.add_subplot(gs[2, 1])
        ax_yaw = fig.add_subplot(gs[2, 2])
        ax_pl  = fig.add_subplot(gs[3, 0])
        ax_pr  = fig.add_subplot(gs[3, 1])
        ax_pd  = fig.add_subplot(gs[3, 2])

        for ax in [ax_map,ax_gx,ax_gy,ax_vx,ax_wz,ax_yaw,ax_pl,ax_pr,ax_pd]:
            ax.set_facecolor(CARD)
            ax.tick_params(colors=AXIS, labelsize=8)
            for spine in ax.spines.values(): spine.set_color(GRID)
            ax.yaxis.label.set_color(AXIS); ax.xaxis.label.set_color(AXIS)
            ax.title.set_color('white')
            ax.grid(True, color=GRID, linewidth=0.5, linestyle='--')

        # MAP
        ax_map.set_title(f'Top-Down Map  ({plan_count} Nav2 replans)')
        ax_map.set_xlabel('X (m)'); ax_map.set_ylabel('Y (m)'); ax_map.set_aspect('equal')

        if plan_first_x:
            ax_map.plot(plan_first_x, plan_first_y, color=BLUE, lw=2.0,
                        linestyle='--', alpha=0.8, label='First Nav2 plan')
        if plan_last_x and plan_count > 1:
            ax_map.plot(plan_last_x, plan_last_y, color=TEAL, lw=1.5,
                        linestyle=':', alpha=0.7, label='Last Nav2 plan')

        if pos_x:
            ax_map.plot(pos_x, pos_y, color=GREEN, lw=1.8, label='Robot path')
            ax_map.scatter(pos_x[0], pos_y[0], color=TEAL, s=100, zorder=6, marker='o', label='Start')
            ax_map.scatter(pos_x[-1], pos_y[-1], color=RED, s=100, zorder=6, marker='x', label='End')
            step = max(1, len(pos_x)//25)
            for i in range(0, len(pos_x), step):
                dx=0.06*math.cos(pos_yaw[i]); dy=0.06*math.sin(pos_yaw[i])
                ax_map.annotate('', xy=(pos_x[i]+dx, pos_y[i]+dy), xytext=(pos_x[i], pos_y[i]),
                                arrowprops=dict(arrowstyle='->', color=GREEN, lw=1.0))

        if goal_x:
            ax_map.scatter(goal_x, goal_y, color=ORG, s=20, alpha=0.5, zorder=4, label='Goal updates')
            ax_map.scatter(goal_x[-1], goal_y[-1], color=ORG, s=150, zorder=6, marker='*', label='Final goal')

        if box_x:
            ax_map.scatter(box_x, box_y, color=PURP, s=12, alpha=0.4, zorder=3, label='Box estimate')
            ax_map.scatter(box_x[-1], box_y[-1], color=PURP, s=120, zorder=5, marker='D', label='Final box')

        ax_map.legend(loc='upper left', facecolor=CARD, edgecolor=GRID, labelcolor='white', fontsize=7)

        # GOAL X
        ax_gx.set_title('Goal X drift'); ax_gx.set_xlabel('Time (s)'); ax_gx.set_ylabel('m')
        if t_goal: ax_gx.plot(t_goal, goal_x, color=ORG, lw=1.2, label='goal_x')
        if t_box:  ax_gx.plot(t_box, box_x, color=PURP, lw=1.0, linestyle='--', alpha=0.7, label='box_x')
        ax_gx.legend(facecolor=CARD, edgecolor=GRID, labelcolor='white', fontsize=7)

        # GOAL Y
        ax_gy.set_title('Goal Y drift'); ax_gy.set_xlabel('Time (s)'); ax_gy.set_ylabel('m')
        if t_goal: ax_gy.plot(t_goal, goal_y, color=YELL, lw=1.2, label='goal_y')
        if t_box:  ax_gy.plot(t_box, box_y, color=PURP, lw=1.0, linestyle='--', alpha=0.7, label='box_y')
        ax_gy.legend(facecolor=CARD, edgecolor=GRID, labelcolor='white', fontsize=7)

        # VX
        ax_vx.set_title('Linear Velocity (vx)'); ax_vx.set_xlabel('Time (s)'); ax_vx.set_ylabel('m/s')
        if t_cmd:  ax_vx.plot(t_cmd, cmd_vx, color=BLUE, lw=1.2, linestyle='--', alpha=0.8, label='cmd_vel')
        if t_odom: ax_vx.plot(t_odom, odom_vx, color=GREEN, lw=1.2, label='measured')
        ax_vx.legend(facecolor=CARD, edgecolor=GRID, labelcolor='white', fontsize=7)

        # WZ
        ax_wz.set_title('Angular Velocity (wz)'); ax_wz.set_xlabel('Time (s)'); ax_wz.set_ylabel('rad/s')
        if t_cmd:  ax_wz.plot(t_cmd, cmd_wz, color=RED, lw=1.2, linestyle='--', alpha=0.8, label='cmd_vel')
        if t_odom: ax_wz.plot(t_odom, odom_wz, color=PURP, lw=1.2, label='measured')
        ax_wz.legend(facecolor=CARD, edgecolor=GRID, labelcolor='white', fontsize=7)

        # YAW
        ax_yaw.set_title('Robot Heading'); ax_yaw.set_xlabel('Time (s)'); ax_yaw.set_ylabel('degrees')
        if t_pos:  ax_yaw.plot(t_pos, [math.degrees(y) for y in pos_yaw], color=TEAL, lw=1.2, label='robot yaw')
        if t_goal: ax_yaw.plot(t_goal, goal_yaw, color=ORG, lw=1.0, linestyle='--', alpha=0.7, label='goal yaw')
        ax_yaw.legend(facecolor=CARD, edgecolor=GRID, labelcolor='white', fontsize=7)

        # PWM LEFT
        ax_pl.set_title('Left Wheel PWM'); ax_pl.set_xlabel('Time (s)'); ax_pl.set_ylabel('PWM')
        ax_pl.set_ylim(-270, 270); ax_pl.axhline(0, color=GRID, lw=0.8)
        if has_pwm: ax_pl.plot(t_pwm, pwm_left, color=ORG, lw=1.2)
        else: ax_pl.text(0.5, 0.5, 'No /motor_pwm', transform=ax_pl.transAxes, color=AXIS, ha='center')

        # PWM RIGHT
        ax_pr.set_title('Right Wheel PWM'); ax_pr.set_xlabel('Time (s)'); ax_pr.set_ylabel('PWM')
        ax_pr.set_ylim(-270, 270); ax_pr.axhline(0, color=GRID, lw=0.8)
        if has_pwm: ax_pr.plot(t_pwm, pwm_right, color=TEAL, lw=1.2)
        else: ax_pr.text(0.5, 0.5, 'No /motor_pwm', transform=ax_pr.transAxes, color=AXIS, ha='center')

        # PWM DIFF
        ax_pd.set_title('PWM Diff (L-R = turning effort)'); ax_pd.set_xlabel('Time (s)'); ax_pd.set_ylabel('PWM diff')
        ax_pd.axhline(0, color=GRID, lw=0.8)
        if has_pwm:
            diff = [l-r for l,r in zip(pwm_left, pwm_right)]
            ax_pd.plot(t_pwm, diff, color=PINK, lw=1.2)
        else:
            ax_pd.text(0.5, 0.5, 'No /motor_pwm', transform=ax_pd.transAxes, color=AXIS, ha='center')

        plot_path = os.path.join(out_dir, 'mission_analysis.png')
        plt.savefig(plot_path, dpi=150, bbox_inches='tight', facecolor=BG)
        print(f"\n  Plot saved: {plot_path}")
        plt.show()
        print(f"\nAll data saved to: {out_dir}")


def main():
    rclpy.init()
    node = MissionRecorder()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\n" + "="*60)
    print("  MISSION RECORDER")
    print("  Topics: /cmd_vel /odom /motor_pwm /goal_update /box_center_pose /plan")
    print("  Press Ctrl+C to stop and plot")
    print("="*60 + "\n")

    try:
        while True:
            time.sleep(1.0)
            with node._lock:
                n_pos=len(node._t_pos); n_cmd=len(node._t_cmd)
                n_pwm=len(node._t_pwm); n_goal=len(node._t_goal); n_plan=node._plan_count
            print(f"\r  Recording... pos={n_pos} cmd={n_cmd} pwm={n_pwm} goals={n_goal} plans={n_plan}  ",
                  end='', flush=True)
    except KeyboardInterrupt:
        print("\n\nStopped. Generating plots...")

    out_dir = f"/tmp/mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    node.save_and_plot(out_dir)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()