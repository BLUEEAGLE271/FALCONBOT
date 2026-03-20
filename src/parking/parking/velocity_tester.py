#!/usr/bin/env python3
"""
Velocity Step Response Tester with PWM + Path Comparison
==========================================================
Commands the robot to a target velocity, records measured response,
AND publishes both the theoretical circular path and actual driven
path to RViz for live comparison.

If vx > 0 and wz > 0: robot should drive a circle of radius = vx/wz
If vx > 0 and wz = 0: straight line

RViz topics:
  /velocity_test/theoretical_path  — blue dashed circle
  /velocity_test/actual_path        — green actual robot path

Usage:
  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
  python3 velocity_tester.py

Requires the C++ controller to publish /motor_pwm as String "left,right"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import threading
import time
import math


class VelocityTester(Node):
    def __init__(self):
        super().__init__('velocity_tester')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # RViz path publishers
        self.theory_path_pub = self.create_publisher(
            Path, '/velocity_test/theoretical_path', 10)
        self.actual_path_pub = self.create_publisher(
            Path, '/velocity_test/actual_path', 10)
        self.radius_marker_pub = self.create_publisher(
            Marker, '/velocity_test/radius_marker', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self.pwm_sub = self.create_subscription(
            String, '/motor_pwm', self._pwm_callback, 10)

        self._lock = threading.Lock()

        # Velocity recording
        self._t_vel        = []
        self._linear_meas  = []
        self._angular_meas = []

        # Position recording (for actual path)
        self._pos_x   = []
        self._pos_y   = []
        self._pos_yaw = []

        # Starting position
        self._start_x   = 0.0
        self._start_y   = 0.0
        self._start_yaw = 0.0
        self._odom_received = False

        # PWM recording
        self._t_pwm      = []
        self._pwm_left   = []
        self._pwm_right  = []

        self._recording  = False
        self._t_start    = 0.0

        # RViz path publishing timer
        self._rviz_timer = self.create_timer(0.2, self._publish_rviz_paths)
        self._theory_path_poses = []   # precomputed once per test
        self._current_frame     = 'odom'

        self.get_logger().info("Velocity Tester ready.")

    # ------------------------------------------------------------------

    def _odom_callback(self, msg):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

        with self._lock:
            if not self._odom_received:
                self._start_x   = x
                self._start_y   = y
                self._start_yaw = yaw
                self._odom_received = True

            if not self._recording:
                # Keep updating start position until recording begins
                self._start_x   = x
                self._start_y   = y
                self._start_yaw = yaw
                return

            t = time.time() - self._t_start
            self._t_vel.append(t)
            self._linear_meas.append(msg.twist.twist.linear.x)
            self._angular_meas.append(msg.twist.twist.angular.z)
            self._pos_x.append(x)
            self._pos_y.append(y)
            self._pos_yaw.append(yaw)

    def _pwm_callback(self, msg):
        if not self._recording:
            return
        try:
            parts = msg.data.split(',')
            left  = float(parts[0])
            right = float(parts[1])
            t = time.time() - self._t_start
            with self._lock:
                self._t_pwm.append(t)
                self._pwm_left.append(left)
                self._pwm_right.append(right)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # THEORETICAL PATH COMPUTATION
    # ------------------------------------------------------------------

    def _compute_theoretical_path(self, vx, wz, duration, start_x, start_y,
                                   start_yaw, dt=0.05):
        """
        Integrate unicycle kinematics forward:
          x_dot   = vx * cos(yaw)
          y_dot   = vx * sin(yaw)
          yaw_dot = wz
        Returns list of (x, y, yaw) at each timestep.
        """
        poses = []
        x, y, yaw = start_x, start_y, start_yaw
        t = 0.0
        while t <= duration:
            poses.append((x, y, yaw))
            x   += vx * math.cos(yaw) * dt
            y   += vx * math.sin(yaw) * dt
            yaw += wz * dt
            t   += dt
        return poses

    def _poses_to_path_msg(self, poses, frame='odom'):
        path              = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame
        for x, y, yaw in poses:
            ps                    = PoseStamped()
            ps.header             = path.header
            ps.pose.position.x    = x
            ps.pose.position.y    = y
            ps.pose.position.z    = 0.0
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        return path

    def _publish_rviz_paths(self):
        """Called at 5Hz — publishes theoretical + actual paths to RViz."""
        with self._lock:
            theory = list(self._theory_path_poses)
            act_x  = list(self._pos_x)
            act_y  = list(self._pos_y)
            act_yaw = list(self._pos_yaw)
            frame  = self._current_frame

        # Theoretical path — blue
        if theory:
            path_msg = self._poses_to_path_msg(theory, frame)
            self.theory_path_pub.publish(path_msg)

        # Actual path — green
        if act_x:
            actual_poses = list(zip(act_x, act_y, act_yaw))
            path_msg = self._poses_to_path_msg(actual_poses, frame)
            self.actual_path_pub.publish(path_msg)

    # ------------------------------------------------------------------
    # TEST RUNNER
    # ------------------------------------------------------------------

    def run_test(self, target_linear, target_angular, duration, ramp_time=0.0):
        print(f"\n🚀 Starting test: vx={target_linear:.3f} m/s | "
              f"wz={target_angular:.3f} rad/s | duration={duration}s")
        if ramp_time > 0:
            print(f"   Ramp time: {ramp_time}s")

        # Print theoretical circle info
        if abs(target_angular) > 1e-6 and abs(target_linear) > 1e-6:
            radius = abs(target_linear / target_angular)
            circum = 2 * math.pi * radius
            period = circum / abs(target_linear)
            print(f"   Theoretical circle: radius={radius:.3f}m | "
                  f"circumference={circum:.3f}m | period={period:.1f}s")
        elif abs(target_angular) < 1e-6:
            dist = target_linear * duration
            print(f"   Theoretical straight line: {dist:.3f}m")

        # Clear buffers
        with self._lock:
            self._t_vel        = []
            self._linear_meas  = []
            self._angular_meas = []
            self._pos_x        = []
            self._pos_y        = []
            self._pos_yaw      = []
            self._t_pwm        = []
            self._pwm_left     = []
            self._pwm_right    = []
            self._theory_path_poses = []
            start_x   = self._start_x
            start_y   = self._start_y
            start_yaw = self._start_yaw

        # Precompute theoretical path from current odom position
        theory = self._compute_theoretical_path(
            target_linear, target_angular, duration,
            start_x, start_y, start_yaw)

        with self._lock:
            self._theory_path_poses = theory

        print(f"   Start pos: ({start_x:.3f}, {start_y:.3f}) | "
              f"yaw={math.degrees(start_yaw):.1f}°")
        print(f"   Watch in RViz:")
        print(f"     Blue  = /velocity_test/theoretical_path")
        print(f"     Green = /velocity_test/actual_path")

        # Brief settle at zero
        self._send_velocity(0.0, 0.0)
        time.sleep(0.5)

        self._t_start   = time.time()
        self._recording = True

        try:
            while (time.time() - self._t_start) < duration:
                elapsed = time.time() - self._t_start

                if ramp_time > 0 and elapsed < ramp_time:
                    factor = elapsed / ramp_time
                    vx = target_linear  * factor
                    wz = target_angular * factor
                else:
                    vx = target_linear
                    wz = target_angular

                self._send_velocity(vx, wz)
                time.sleep(0.02)  # 50Hz

        finally:
            self._recording = False
            self._send_velocity(0.0, 0.0)
            print("🛑 Test complete — robot stopped.")

        with self._lock:
            t_vel        = list(self._t_vel)
            linear_meas  = list(self._linear_meas)
            angular_meas = list(self._angular_meas)
            pos_x        = list(self._pos_x)
            pos_y        = list(self._pos_y)
            pos_yaw      = list(self._pos_yaw)
            t_pwm        = list(self._t_pwm)
            pwm_left     = list(self._pwm_left)
            pwm_right    = list(self._pwm_right)

        if len(t_vel) < 5:
            print("⚠️  Not enough odom data. Is /odom publishing?")
            return

        self._analyse_and_plot(
            t_vel, linear_meas, angular_meas,
            pos_x, pos_y,
            theory,
            t_pwm, pwm_left, pwm_right,
            target_linear, target_angular, duration, ramp_time,
            start_x, start_y
        )

    def _send_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    # ANALYSIS + PLOTTING
    # ------------------------------------------------------------------

    def _analyse_and_plot(self, t_vel, linear_meas, angular_meas,
                           pos_x, pos_y,
                           theory_poses,
                           t_pwm, pwm_left, pwm_right,
                           target_linear, target_angular, duration, ramp_time,
                           start_x, start_y):

        t_vel        = np.array(t_vel)
        linear_meas  = np.array(linear_meas)
        angular_meas = np.array(angular_meas)
        pos_x        = np.array(pos_x)
        pos_y        = np.array(pos_y)

        has_pwm = len(t_pwm) > 2
        if has_pwm:
            t_pwm     = np.array(t_pwm)
            pwm_left  = np.array(pwm_left)
            pwm_right = np.array(pwm_right)

        # Build target signal
        if ramp_time > 0:
            linear_target  = np.where(t_vel < ramp_time,
                                       target_linear  * t_vel / ramp_time,
                                       target_linear)
            angular_target = np.where(t_vel < ramp_time,
                                       target_angular * t_vel / ramp_time,
                                       target_angular)
        else:
            linear_target  = np.full_like(t_vel, target_linear)
            angular_target = np.full_like(t_vel, target_angular)

        # Theoretical path arrays
        theory_x = np.array([p[0] for p in theory_poses])
        theory_y = np.array([p[1] for p in theory_poses])

        # Path error — distance from actual to nearest theory point
        path_errors = []
        for ax, ay in zip(pos_x, pos_y):
            dists = np.hypot(theory_x - ax, theory_y - ay)
            path_errors.append(dists.min())
        path_errors = np.array(path_errors)

        # Metrics
        def analyse(meas, target, label):
            if abs(target) < 1e-6:
                return
            print(f"\n📊 {label} Analysis (target={target:.3f}):")
            ss_idx = int(len(meas) * 0.8)
            steady_state = np.mean(meas[ss_idx:])
            ss_error = abs(target - steady_state)
            print(f"   Steady-state value:  {steady_state:.4f}")
            print(f"   Steady-state error:  {ss_error:.4f}  "
                  f"({ss_error/abs(target)*100:.1f}%)")
            idx_10 = next((i for i, v in enumerate(meas)
                           if abs(v) >= 0.10 * abs(target)), None)
            idx_90 = next((i for i, v in enumerate(meas)
                           if abs(v) >= 0.90 * abs(target)), None)
            if idx_10 is not None and idx_90 is not None:
                print(f"   Rise time (10→90%): {t_vel[idx_90] - t_vel[idx_10]:.3f}s")
            peak = np.max(np.abs(meas))
            overshoot = ((peak - abs(target)) / abs(target)) * 100
            print(f"   Peak value:         {peak:.4f}")
            print(f"   Overshoot:          {overshoot:.1f}%" if overshoot > 0
                  else f"   Overshoot:          None")

        analyse(linear_meas,  target_linear,  "Linear  (vx)")
        analyse(angular_meas, target_angular, "Angular (wz)")

        print(f"\n📊 Path Tracking:")
        print(f"   Mean path error: {np.mean(path_errors):.4f}m")
        print(f"   Max  path error: {np.max(path_errors):.4f}m")

        if has_pwm:
            print(f"\n📊 PWM Summary:")
            print(f"   Left  — mean: {np.mean(pwm_left):.1f}  "
                  f"max: {np.max(np.abs(pwm_left)):.1f}")
            print(f"   Right — mean: {np.mean(pwm_right):.1f}  "
                  f"max: {np.max(np.abs(pwm_right)):.1f}")

        # ── PLOT ──────────────────────────────────────────────────────
        BG   = '#0d1117'; CARD = '#161b22'; GRID = '#21262d'
        AXIS = '#8b949e'; BLUE = '#58a6ff'; GREEN = '#3fb950'
        RED  = '#f78166'; PURP = '#d2a8ff'; ORG  = '#ffa657'
        TEAL = '#39d353'; PINK = '#ff7eb6'

        fig = plt.figure(figsize=(18, 12), facecolor=BG)
        fig.suptitle(
            f'Velocity Test — vx={target_linear:.3f} m/s  '
            f'wz={target_angular:.3f} rad/s',
            color='white', fontsize=13, fontweight='bold', y=0.99)

        gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.5, wspace=0.38)

        # Row 0: path comparison (large), path error
        ax_path = fig.add_subplot(gs[0:2, 0:2])   # big path plot
        ax_perr = fig.add_subplot(gs[0, 2])        # path error over time
        ax_vel_err = fig.add_subplot(gs[1, 2])     # velocity error

        # Row 2: vx, wz, pwm
        ax_vx  = fig.add_subplot(gs[2, 0])
        ax_wz  = fig.add_subplot(gs[2, 1])
        ax_pwm = fig.add_subplot(gs[2, 2])

        all_axes = [ax_path, ax_perr, ax_vel_err, ax_vx, ax_wz, ax_pwm]
        for ax in all_axes:
            ax.set_facecolor(CARD)
            ax.tick_params(colors=AXIS, labelsize=8)
            for sp in ax.spines.values(): sp.set_color(GRID)
            ax.yaxis.label.set_color(AXIS)
            ax.xaxis.label.set_color(AXIS)
            ax.title.set_color('white')
            ax.grid(True, color=GRID, linewidth=0.5, linestyle='--')

        # ── PATH COMPARISON ───────────────────────────────────────────
        ax_path.set_title('Path Comparison: Theoretical vs Actual')
        ax_path.set_xlabel('X (m)'); ax_path.set_ylabel('Y (m)')
        ax_path.set_aspect('equal')

        # Theoretical path — blue dashed
        ax_path.plot(theory_x, theory_y, color=BLUE, lw=2.0,
                     linestyle='--', alpha=0.9, label='Theoretical path')

        # Actual path — green solid
        if len(pos_x) > 0:
            ax_path.plot(pos_x, pos_y, color=GREEN, lw=2.0,
                         alpha=0.9, label='Actual path')
            ax_path.scatter(pos_x[0], pos_y[0], color=TEAL, s=100,
                            zorder=6, marker='o', label='Start')
            ax_path.scatter(pos_x[-1], pos_y[-1], color=RED, s=100,
                            zorder=6, marker='x', label='End')

        # Draw theoretical circle centre if turning
        if abs(target_angular) > 1e-6 and abs(target_linear) > 1e-6:
            radius = target_linear / target_angular
            # Circle centre is perpendicular to start heading
            cx = start_x - radius * math.sin(
                math.atan2(theory_y[1]-theory_y[0], theory_x[1]-theory_x[0])
                + math.pi/2)
            cy = start_y + radius * math.cos(
                math.atan2(theory_y[1]-theory_y[0], theory_x[1]-theory_x[0])
                + math.pi/2)
            ax_path.scatter(cx, cy, color=BLUE, s=60, zorder=5,
                            marker='+', alpha=0.6, label=f'Circle centre')
            ax_path.annotate(f'r={abs(radius):.2f}m',
                             xy=(cx, cy), color=BLUE, fontsize=7,
                             ha='center', va='bottom')

        ax_path.legend(loc='upper left', facecolor=CARD,
                       edgecolor=GRID, labelcolor='white', fontsize=7)

        # ── PATH ERROR ────────────────────────────────────────────────
        ax_perr.set_title('Path Error (actual→theory)')
        ax_perr.set_xlabel('Time (s)'); ax_perr.set_ylabel('metres')
        ax_perr.axhline(0.05, color=RED, lw=0.8, linestyle=':',
                        alpha=0.7, label='5cm threshold')
        ax_perr.plot(t_vel[:len(path_errors)], path_errors,
                     color=PINK, lw=1.2)
        ax_perr.fill_between(t_vel[:len(path_errors)], path_errors,
                              alpha=0.15, color=PINK)
        ax_perr.legend(facecolor=CARD, edgecolor=GRID,
                       labelcolor='white', fontsize=7)

        # ── VELOCITY ERROR ────────────────────────────────────────────
        ax_vel_err.set_title('Velocity Error (target - measured)')
        ax_vel_err.set_xlabel('Time (s)'); ax_vel_err.set_ylabel('error')
        ax_vel_err.axhline(0, color=GRID, lw=0.8)
        lin_err = linear_target  - linear_meas
        ang_err = angular_target - angular_meas
        ax_vel_err.plot(t_vel, lin_err,  color=BLUE, lw=1.1, label='vx error')
        ax_vel_err.plot(t_vel, ang_err, color=RED,  lw=1.1, label='wz error')
        ax_vel_err.legend(facecolor=CARD, edgecolor=GRID,
                          labelcolor='white', fontsize=7)

        # ── LINEAR VELOCITY ───────────────────────────────────────────
        ax_vx.set_title('Linear Velocity (vx)')
        ax_vx.set_xlabel('Time (s)'); ax_vx.set_ylabel('m/s')
        ax_vx.plot(t_vel, linear_target, color=BLUE, lw=1.5,
                   linestyle='--', alpha=0.8,
                   label=f'Target ({target_linear:.3f} m/s)')
        ax_vx.plot(t_vel, linear_meas, color=GREEN, lw=1.5, label='Measured')
        ax_vx.legend(loc='lower right', facecolor=CARD,
                     edgecolor=GRID, labelcolor='white', fontsize=7)

        # ── ANGULAR VELOCITY ──────────────────────────────────────────
        ax_wz.set_title('Angular Velocity (wz)')
        ax_wz.set_xlabel('Time (s)'); ax_wz.set_ylabel('rad/s')
        ax_wz.plot(t_vel, angular_target, color=RED, lw=1.5,
                   linestyle='--', alpha=0.8,
                   label=f'Target ({target_angular:.3f} rad/s)')
        ax_wz.plot(t_vel, angular_meas, color=PURP, lw=1.5, label='Measured')
        ax_wz.legend(loc='lower right', facecolor=CARD,
                     edgecolor=GRID, labelcolor='white', fontsize=7)

        # ── PWM ───────────────────────────────────────────────────────
        ax_pwm.set_title('Wheel PWM')
        ax_pwm.set_xlabel('Time (s)'); ax_pwm.set_ylabel('PWM')
        ax_pwm.set_ylim(-270, 270); ax_pwm.axhline(0, color=GRID, lw=0.8)
        if has_pwm:
            ax_pwm.plot(t_pwm, pwm_left,  color=ORG,  lw=1.2, label='Left')
            ax_pwm.plot(t_pwm, pwm_right, color=TEAL, lw=1.2, label='Right')
            ax_pwm.legend(facecolor=CARD, edgecolor=GRID,
                          labelcolor='white', fontsize=7)
        else:
            ax_pwm.text(0.5, 0.5, 'No /motor_pwm',
                        transform=ax_pwm.transAxes,
                        color=AXIS, ha='center')

        plt.savefig('/tmp/velocity_response.png', dpi=150,
                    bbox_inches='tight', facecolor=BG)
        print("\n📈 Plot saved to /tmp/velocity_response.png")
        plt.show()


def get_float(prompt, default=None):
    while True:
        try:
            val = input(prompt).strip()
            if val == '' and default is not None:
                return default
            return float(val)
        except ValueError:
            print("   Please enter a number.")


def main():
    rclpy.init()
    node = VelocityTester()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\n" + "="*60)
    print("  VELOCITY STEP RESPONSE TESTER  (+ Path Comparison)")
    print("  Requires /odom, /cmd_vel, /motor_pwm")
    print("")
    print("  Add to RViz:")
    print("    Path → /velocity_test/theoretical_path  (blue)")
    print("    Path → /velocity_test/actual_path        (green)")
    print("="*60)

    try:
        while True:
            print("\n--- New Test ---")
            print("  Enter 0 for both velocities to exit\n")

            vx       = get_float("  Target linear velocity  (m/s)   [0.3]: ", 0.3)
            wz       = get_float("  Target angular velocity (rad/s) [0.0]: ", 0.0)
            duration = get_float("  Test duration           (s)      [5.0]: ", 5.0)
            ramp     = get_float("  Ramp time (0=step, >0=ramp)      [0.0]: ", 0.0)

            if abs(vx) < 1e-6 and abs(wz) < 1e-6:
                print("\n  Both velocities are 0 — exiting.")
                break

            print("\n  ⚠️  Robot will move! Make sure area is clear.")
            confirm = input("  Press ENTER to start, or 'n' to cancel: ").strip()
            if confirm.lower() == 'n':
                print("  Cancelled.")
                continue

            node.run_test(vx, wz, duration, ramp)

            again = input("\n  Run another test? (y/n) [y]: ").strip()
            if again.lower() == 'n':
                break

    except KeyboardInterrupt:
        pass
    finally:
        node._send_velocity(0.0, 0.0)
        print("\n🛑 Emergency stop sent.")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()