#!/usr/bin/env python3
"""
Path Tracking Diagnostic Recorder
====================================
Diagnoses whether tracking failure is caused by:
  A) MPPI controller tuning  — plan vs cmd_vel mismatch
  B) Velocity controller     — cmd_vel vs actual velocity mismatch
  C) Both

Records:
  /plan                  — Nav2 planned path
  /trajectories          — MPPI optimal trajectory
  /cmd_vel               — what MPPI commands the robot to do
  /odom                  — actual velocity + position
  /motor_pwm             — actual PWM sent to wheels
  /goal_update           — goal poses from box_estimator
  /visual_target_goal    — rough + final goals
  /local_costmap/costmap — what local planner sees
  /tf                    — odom→map transform drift

Saves to: ~/ros2_ws/mission_diagnostics/diag_<timestamp>/

Usage:
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  python3 tracking_diagnostics.py

Press Ctrl+C to stop and generate diagnostic plots.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
import tf2_ros
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import threading
import time
import os
import csv
import math
from datetime import datetime


class TrackingDiagnostics(Node):
    def __init__(self):
        super().__init__('tracking_diagnostics')

        be_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._lock = threading.Lock()
        self._t0   = time.time()

        # --- cmd_vel ---
        self._t_cmd, self._cmd_vx, self._cmd_wz = [], [], []

        # --- odom velocity ---
        self._t_odom, self._odom_vx, self._odom_wz = [], [], []

        # --- robot position ---
        self._t_pos, self._pos_x, self._pos_y, self._pos_yaw = [], [], [], []

        # --- PWM ---
        self._t_pwm, self._pwm_left, self._pwm_right = [], [], []

        # --- Nav2 plan (keep all received plans with timestamps) ---
        self._plans = []          # list of (t, [x...], [y...])

        # --- MPPI optimal trajectory (keep all) ---
        self._mppi_trajs = []     # list of (t, [x...], [y...])

        # --- goal poses ---
        self._t_goal, self._goal_x, self._goal_y, self._goal_yaw = [], [], [], []

        # --- path tracking error (cross-track error) ---
        self._t_cte, self._cte = [], []
        self._current_plan_x = []
        self._current_plan_y = []

        # --- velocity tracking errors ---
        self._t_vel_err, self._vx_err, self._wz_err = [], [], []
        self._latest_cmd_vx  = 0.0
        self._latest_cmd_wz  = 0.0

        # --- Subscribers ---
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, be_qos)
        self.create_subscription(String, '/motor_pwm', self._pwm_cb, 10)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.create_subscription(PoseStamped, '/goal_update', self._goal_cb, 10)
        self.create_subscription(PoseStamped, '/visual_target_goal', self._goal_cb, 10)

        # MPPI publishes optimal trajectory as MarkerArray on /trajectories
        # or as Path on /local_plan — try both
        self.create_subscription(Path, '/local_plan', self._mppi_cb, be_qos)

        # TF buffer to track odom→map drift
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer to record odom→map drift at 5Hz
        self.create_timer(0.2, self._tf_drift_cb)
        self._t_drift, self._drift_x, self._drift_y = [], [], []

        self.get_logger().info("Tracking Diagnostics started. Press Ctrl+C to stop.")

    # ------------------------------------------------------------------
    def _t(self):
        return time.time() - self._t0

    def _cmd_cb(self, msg):
        t = self._t()
        with self._lock:
            self._t_cmd.append(t)
            self._cmd_vx.append(msg.linear.x)
            self._cmd_wz.append(msg.angular.z)
            self._latest_cmd_vx = msg.linear.x
            self._latest_cmd_wz = msg.angular.z

    def _odom_cb(self, msg):
        t   = self._t()
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

            # Velocity tracking error (cmd vs measured)
            vx_err = self._latest_cmd_vx - vx
            wz_err = self._latest_cmd_wz - wz
            self._t_vel_err.append(t)
            self._vx_err.append(vx_err)
            self._wz_err.append(wz_err)

            # Cross-track error (distance from robot to nearest plan point)
            if self._current_plan_x:
                cte = self._compute_cte(x, y,
                                        self._current_plan_x,
                                        self._current_plan_y)
                self._t_cte.append(t)
                self._cte.append(cte)

    def _pwm_cb(self, msg):
        t = self._t()
        try:
            parts = msg.data.split(',')
            l, r  = float(parts[0]), float(parts[1])
            with self._lock:
                self._t_pwm.append(t); self._pwm_left.append(l); self._pwm_right.append(r)
        except Exception:
            pass

    def _plan_cb(self, msg):
        t  = self._t()
        xs = [p.pose.position.x for p in msg.poses]
        ys = [p.pose.position.y for p in msg.poses]
        with self._lock:
            self._plans.append((t, xs, ys))
            self._current_plan_x = xs
            self._current_plan_y = ys

    def _mppi_cb(self, msg):
        t  = self._t()
        xs = [p.pose.position.x for p in msg.poses]
        ys = [p.pose.position.y for p in msg.poses]
        with self._lock:
            self._mppi_trajs.append((t, xs, ys))

    def _goal_cb(self, msg):
        t   = self._t()
        q   = msg.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        with self._lock:
            self._t_goal.append(t)
            self._goal_x.append(msg.pose.position.x)
            self._goal_y.append(msg.pose.position.y)
            self._goal_yaw.append(math.degrees(yaw))

    def _tf_drift_cb(self):
        """Record the odom→map TF offset over time."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'odom',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.05)
            )
            t  = self._t()
            dx = tf.transform.translation.x
            dy = tf.transform.translation.y
            with self._lock:
                self._t_drift.append(t)
                self._drift_x.append(dx)
                self._drift_y.append(dy)
        except Exception:
            pass

    def _compute_cte(self, rx, ry, plan_x, plan_y):
        """Cross-track error: distance from robot to nearest point on plan."""
        if not plan_x:
            return 0.0
        min_dist = float('inf')
        for px, py in zip(plan_x, plan_y):
            d = math.hypot(rx - px, ry - py)
            if d < min_dist:
                min_dist = d
        return min_dist

    # ------------------------------------------------------------------
    def save_and_plot(self, out_dir):
        os.makedirs(out_dir, exist_ok=True)

        with self._lock:
            t_cmd     = list(self._t_cmd);     cmd_vx    = list(self._cmd_vx)
            cmd_wz    = list(self._cmd_wz)
            t_odom    = list(self._t_odom);    odom_vx   = list(self._odom_vx)
            odom_wz   = list(self._odom_wz)
            t_pos     = list(self._t_pos);     pos_x     = list(self._pos_x)
            pos_y     = list(self._pos_y);     pos_yaw   = list(self._pos_yaw)
            t_pwm     = list(self._t_pwm);     pwm_left  = list(self._pwm_left)
            pwm_right = list(self._pwm_right)
            plans     = list(self._plans)
            mppi_tjs  = list(self._mppi_trajs)
            t_goal    = list(self._t_goal);    goal_x    = list(self._goal_x)
            goal_y    = list(self._goal_y);    goal_yaw  = list(self._goal_yaw)
            t_cte     = list(self._t_cte);     cte       = list(self._cte)
            t_ve      = list(self._t_vel_err); vx_err    = list(self._vx_err)
            wz_err    = list(self._wz_err)
            t_drift   = list(self._t_drift);   drift_x   = list(self._drift_x)
            drift_y   = list(self._drift_y)

        # --- Save CSVs ---
        def save_csv(fname, headers, *cols):
            if not cols[0]: return
            p = os.path.join(out_dir, fname)
            with open(p, 'w', newline='') as f:
                w = csv.writer(f); w.writerow(headers)
                for row in zip(*cols): w.writerow(row)
            print(f"  saved {p}")

        save_csv('cmd_vel.csv',
                 ['t','cmd_vx','cmd_wz'], t_cmd, cmd_vx, cmd_wz)
        save_csv('odom.csv',
                 ['t','odom_vx','odom_wz','x','y','yaw'],
                 t_odom, odom_vx, odom_wz, t_pos, pos_x, pos_y)
        save_csv('pwm.csv',
                 ['t','left','right'], t_pwm, pwm_left, pwm_right)
        save_csv('cte.csv',
                 ['t','cross_track_error_m'], t_cte, cte)
        save_csv('velocity_error.csv',
                 ['t','vx_error','wz_error'], t_ve, vx_err, wz_err)
        save_csv('tf_drift.csv',
                 ['t','odom_map_dx','odom_map_dy'], t_drift, drift_x, drift_y)
        save_csv('goals.csv',
                 ['t','goal_x','goal_y','goal_yaw_deg'],
                 t_goal, goal_x, goal_y, goal_yaw)

        # Save all plans
        plans_path = os.path.join(out_dir, 'plans.csv')
        with open(plans_path, 'w', newline='') as f:
            w = csv.writer(f); w.writerow(['plan_t', 'x', 'y'])
            for pt, pxs, pys in plans:
                for px, py in zip(pxs, pys):
                    w.writerow([pt, px, py])
        print(f"  saved {plans_path}")

        # ------------------------------------------------------------------
        # DIAGNOSTIC VERDICT
        # ------------------------------------------------------------------
        print("\n" + "="*60)
        print("  DIAGNOSTIC VERDICT")
        print("="*60)

        if cte:
            avg_cte = np.mean(cte)
            max_cte = np.max(cte)
            print(f"\n  Cross-Track Error (CTE):")
            print(f"    Mean: {avg_cte:.4f}m   Max: {max_cte:.4f}m")
            if avg_cte > 0.15:
                print(f"    ⚠️  HIGH CTE — robot is NOT following the Nav2 plan")
                print(f"    → Likely cause: MPPI tuning OR velocity controller")
            else:
                print(f"    ✅ CTE acceptable — robot follows the plan well")

        if vx_err and odom_vx:
            avg_vx_err = np.mean(np.abs(vx_err))
            avg_wz_err = np.mean(np.abs(wz_err))
            print(f"\n  Velocity Tracking Error:")
            print(f"    Mean |vx error|: {avg_vx_err:.4f} m/s")
            print(f"    Mean |wz error|: {avg_wz_err:.4f} rad/s")
            if avg_vx_err > 0.05:
                print(f"    ⚠️  HIGH VX ERROR — velocity controller not tracking cmd_vel")
                print(f"    → Tune kp_v / ki_v in lidar_velocity_controller")
            else:
                print(f"    ✅ Velocity tracking good")
            if avg_wz_err > 0.1:
                print(f"    ⚠️  HIGH WZ ERROR — angular controller not tracking cmd_vel")
                print(f"    → Tune kp_w / ki_w in lidar_velocity_controller")
            else:
                print(f"    ✅ Angular tracking good")

        if drift_x:
            max_drift = max(math.hypot(dx, dy) for dx, dy in zip(drift_x, drift_y))
            print(f"\n  SLAM/TF Drift:")
            print(f"    Max odom→map offset: {max_drift:.4f}m")
            if max_drift > 0.1:
                print(f"    ⚠️  SLAM drift detected — odom frame moving relative to map")
                print(f"    → Goals in odom frame will appear to drift in map frame")
            else:
                print(f"    ✅ SLAM stable")

        if cte and vx_err:
            high_cte = np.mean(cte) > 0.15
            high_vel = np.mean(np.abs(vx_err)) > 0.05
            print(f"\n  ROOT CAUSE SUMMARY:")
            if high_cte and high_vel:
                print(f"    BOTH MPPI and velocity controller contributing to tracking failure")
                print(f"    Fix velocity controller first, then re-tune MPPI")
            elif high_cte and not high_vel:
                print(f"    Velocity controller is fine — MPPI critics need tuning")
                print(f"    Robot follows velocity commands but MPPI picks wrong commands")
                print(f"    → Raise PathFollowCritic.cost_weight")
                print(f"    → Raise PathAlignCritic.cost_weight")
            elif not high_cte and high_vel:
                print(f"    MPPI is fine — velocity controller is the problem")
                print(f"    MPPI sends correct commands but robot doesn't execute them")
                print(f"    → Tune kp_v, ki_v, kp_w, ki_w")
            else:
                print(f"    ✅ Both systems performing well")
        print("="*60)

        # ------------------------------------------------------------------
        # PLOTS
        # ------------------------------------------------------------------
        BG='#0d1117'; CARD='#161b22'; GRID='#21262d'; AXIS='#8b949e'
        BLUE='#58a6ff'; GREEN='#3fb950'; RED='#f78166'; PURP='#d2a8ff'
        ORG='#ffa657'; TEAL='#39d353'; YELL='#e3b341'; PINK='#ff7eb6'
        CYAN='#79c0ff'

        fig = plt.figure(figsize=(22, 18), facecolor=BG)
        fig.suptitle('Path Tracking Diagnostics — MPPI vs Velocity Controller',
                     color='white', fontsize=14, fontweight='bold', y=0.99)

        gs = gridspec.GridSpec(4, 3, figure=fig, hspace=0.55, wspace=0.38)

        # Row 0+1: big map
        ax_map = fig.add_subplot(gs[0:2, 0:2])
        ax_cte = fig.add_subplot(gs[0, 2])
        ax_dr  = fig.add_subplot(gs[1, 2])

        # Row 2: vx, wz, pwm diff
        ax_vx  = fig.add_subplot(gs[2, 0])
        ax_wz  = fig.add_subplot(gs[2, 1])
        ax_pd  = fig.add_subplot(gs[2, 2])

        # Row 3: vx error, wz error, pwm left+right
        ax_ve  = fig.add_subplot(gs[3, 0])
        ax_we  = fig.add_subplot(gs[3, 1])
        ax_pwm = fig.add_subplot(gs[3, 2])

        for ax in [ax_map,ax_cte,ax_dr,ax_vx,ax_wz,ax_pd,ax_ve,ax_we,ax_pwm]:
            ax.set_facecolor(CARD)
            ax.tick_params(colors=AXIS, labelsize=8)
            for sp in ax.spines.values(): sp.set_color(GRID)
            ax.yaxis.label.set_color(AXIS); ax.xaxis.label.set_color(AXIS)
            ax.title.set_color('white')
            ax.grid(True, color=GRID, linewidth=0.5, linestyle='--')

        # ── MAP ──────────────────────────────────────────────────
        ax_map.set_title(f'Top-Down: Plan vs Robot Path ({len(plans)} Nav2 replans)')
        ax_map.set_xlabel('X (m)'); ax_map.set_ylabel('Y (m)')
        ax_map.set_aspect('equal')

        # All Nav2 plans
        for i, (pt, pxs, pys) in enumerate(plans):
            alpha = 0.3 + 0.7 * (i / max(len(plans)-1, 1))
            lbl   = 'Nav2 plan' if i == 0 else None
            ax_map.plot(pxs, pys, color=BLUE, lw=1.5, linestyle='--',
                        alpha=alpha, label=lbl)

        # Latest MPPI trajectories (last 5)
        for i, (mt, mxs, mys) in enumerate(mppi_tjs[-5:]):
            lbl = 'MPPI trajectory' if i == 0 else None
            ax_map.plot(mxs, mys, color=YELL, lw=1.0, alpha=0.4, label=lbl)

        # Robot actual path
        if pos_x:
            ax_map.plot(pos_x, pos_y, color=GREEN, lw=2.0, label='Robot path')
            ax_map.scatter(pos_x[0], pos_y[0], color=TEAL, s=100, zorder=6,
                           marker='o', label='Start')
            ax_map.scatter(pos_x[-1], pos_y[-1], color=RED, s=100, zorder=6,
                           marker='x', label='End')
            step = max(1, len(pos_x)//20)
            for i in range(0, len(pos_x), step):
                dx2 = 0.05*math.cos(pos_yaw[i]); dy2 = 0.05*math.sin(pos_yaw[i])
                ax_map.annotate('', xy=(pos_x[i]+dx2, pos_y[i]+dy2),
                                xytext=(pos_x[i], pos_y[i]),
                                arrowprops=dict(arrowstyle='->', color=GREEN, lw=1.0))

        if goal_x:
            ax_map.scatter(goal_x[-1], goal_y[-1], color=ORG, s=180,
                           zorder=7, marker='*', label='Goal')

        ax_map.legend(loc='upper left', facecolor=CARD, edgecolor=GRID,
                      labelcolor='white', fontsize=7)

        # ── CROSS-TRACK ERROR ────────────────────────────────────
        ax_cte.set_title('Cross-Track Error (robot→plan)')
        ax_cte.set_xlabel('Time (s)'); ax_cte.set_ylabel('metres')
        ax_cte.axhline(0.10, color=RED, lw=0.8, linestyle=':', alpha=0.7,
                       label='0.10m threshold')
        if t_cte:
            ax_cte.plot(t_cte, cte, color=PINK, lw=1.2)
            ax_cte.fill_between(t_cte, cte, alpha=0.15, color=PINK)
        ax_cte.legend(facecolor=CARD, edgecolor=GRID, labelcolor='white', fontsize=7)

        # ── SLAM DRIFT ───────────────────────────────────────────
        ax_dr.set_title('SLAM Drift (odom→map offset)')
        ax_dr.set_xlabel('Time (s)'); ax_dr.set_ylabel('metres')
        if t_drift:
            drift_mag = [math.hypot(dx, dy) for dx, dy in zip(drift_x, drift_y)]
            ax_dr.plot(t_drift, drift_mag, color=CYAN, lw=1.2, label='|drift|')
            ax_dr.plot(t_drift, drift_x, color=BLUE, lw=0.8, linestyle='--', label='dx')
            ax_dr.plot(t_drift, drift_y, color=ORG, lw=0.8, linestyle='--', label='dy')
        ax_dr.legend(facecolor=CARD, edgecolor=GRID, labelcolor='white', fontsize=7)

        # ── LINEAR VELOCITY ───────────────────────────────────────
        ax_vx.set_title('Linear Velocity (vx)')
        ax_vx.set_xlabel('Time (s)'); ax_vx.set_ylabel('m/s')
        if t_cmd:  ax_vx.plot(t_cmd, cmd_vx, color=BLUE, lw=1.2,
                              linestyle='--', alpha=0.8, label='cmd_vel (MPPI)')
        if t_odom: ax_vx.plot(t_odom, odom_vx, color=GREEN, lw=1.2, label='measured')
        ax_vx.legend(facecolor=CARD, edgecolor=GRID, labelcolor='white', fontsize=7)

        # ── ANGULAR VELOCITY ──────────────────────────────────────
        ax_wz.set_title('Angular Velocity (wz)')
        ax_wz.set_xlabel('Time (s)'); ax_wz.set_ylabel('rad/s')
        if t_cmd:  ax_wz.plot(t_cmd, cmd_wz, color=RED, lw=1.2,
                              linestyle='--', alpha=0.8, label='cmd_vel (MPPI)')
        if t_odom: ax_wz.plot(t_odom, odom_wz, color=PURP, lw=1.2, label='measured')
        ax_wz.legend(facecolor=CARD, edgecolor=GRID, labelcolor='white', fontsize=7)

        # ── PWM DIFF ──────────────────────────────────────────────
        ax_pd.set_title('PWM Diff (L-R = turning effort)')
        ax_pd.set_xlabel('Time (s)'); ax_pd.set_ylabel('PWM diff')
        ax_pd.axhline(0, color=GRID, lw=0.8)
        if t_pwm:
            diff = [l-r for l,r in zip(pwm_left, pwm_right)]
            ax_pd.plot(t_pwm, diff, color=PINK, lw=1.2)

        # ── VX ERROR ──────────────────────────────────────────────
        ax_ve.set_title('Linear Velocity Error (cmd - measured)\n'
                        'If large → velocity controller problem')
        ax_ve.set_xlabel('Time (s)'); ax_ve.set_ylabel('m/s error')
        ax_ve.axhline(0, color=GRID, lw=0.8)
        ax_ve.axhline( 0.05, color=RED, lw=0.7, linestyle=':', alpha=0.6)
        ax_ve.axhline(-0.05, color=RED, lw=0.7, linestyle=':', alpha=0.6,
                      label='±0.05 threshold')
        if t_ve: ax_ve.plot(t_ve, vx_err, color=GREEN, lw=1.1)
        ax_ve.legend(facecolor=CARD, edgecolor=GRID, labelcolor='white', fontsize=7)

        # ── WZ ERROR ──────────────────────────────────────────────
        ax_we.set_title('Angular Velocity Error (cmd - measured)\n'
                        'If large → velocity controller problem')
        ax_we.set_xlabel('Time (s)'); ax_we.set_ylabel('rad/s error')
        ax_we.axhline(0, color=GRID, lw=0.8)
        ax_we.axhline( 0.1, color=RED, lw=0.7, linestyle=':', alpha=0.6)
        ax_we.axhline(-0.1, color=RED, lw=0.7, linestyle=':', alpha=0.6,
                      label='±0.1 threshold')
        if t_ve: ax_we.plot(t_ve, wz_err, color=PURP, lw=1.1)
        ax_we.legend(facecolor=CARD, edgecolor=GRID, labelcolor='white', fontsize=7)

        # ── PWM ───────────────────────────────────────────────────
        ax_pwm.set_title('Wheel PWM')
        ax_pwm.set_xlabel('Time (s)'); ax_pwm.set_ylabel('PWM')
        ax_pwm.set_ylim(-270, 270); ax_pwm.axhline(0, color=GRID, lw=0.8)
        if t_pwm:
            ax_pwm.plot(t_pwm, pwm_left,  color=ORG,  lw=1.0, label='Left')
            ax_pwm.plot(t_pwm, pwm_right, color=TEAL, lw=1.0, label='Right')
        ax_pwm.legend(facecolor=CARD, edgecolor=GRID, labelcolor='white', fontsize=7)

        plot_path = os.path.join(out_dir, 'tracking_diagnostics.png')
        plt.savefig(plot_path, dpi=150, bbox_inches='tight', facecolor=BG)
        print(f"\n  Plot saved: {plot_path}")
        plt.show()
        print(f"\nAll data saved to: {out_dir}")


def main():
    rclpy.init()
    node = TrackingDiagnostics()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Save to ros2_ws folder
    home     = os.path.expanduser('~')
    base_dir = os.path.join(home, 'ros2_ws', 'mission_diagnostics')
    os.makedirs(base_dir, exist_ok=True)

    print("\n" + "="*65)
    print("  PATH TRACKING DIAGNOSTICS")
    print("  Topics: /plan /cmd_vel /odom /motor_pwm /trajectories")
    print("          /local_plan /goal_update /visual_target_goal /tf")
    print(f"  Saving to: {base_dir}")
    print("  Press Ctrl+C to stop and generate diagnostic report")
    print("="*65 + "\n")

    try:
        while True:
            time.sleep(1.0)
            with node._lock:
                n_pos  = len(node._t_pos)
                n_cmd  = len(node._t_cmd)
                n_pwm  = len(node._t_pwm)
                n_plan = len(node._plans)
                n_mppi = len(node._mppi_trajs)
                n_cte  = len(node._t_cte)
                cte_now = f"{node._cte[-1]:.3f}m" if node._cte else "n/a"
            print(f"\r  pos={n_pos} cmd={n_cmd} pwm={n_pwm} "
                  f"plans={n_plan} mppi={n_mppi} "
                  f"cte_samples={n_cte} latest_cte={cte_now}  ",
                  end='', flush=True)
    except KeyboardInterrupt:
        print("\n\nStopped. Generating diagnostic report...")

    ts      = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.join(base_dir, f'diag_{ts}')
    node.save_and_plot(out_dir)

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()