#!/usr/bin/env python3
"""
Velocity Step Response Tester with PWM Recording
=================================================
Commands the robot to a target velocity, records measured response
AND the PWM sent to each wheel, then plots everything together.

Usage:
  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
  python3 velocity_tester.py

Requires the C++ controller to publish /motor_pwm as String "left,right"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import threading
import time


class VelocityTester(Node):
    def __init__(self):
        super().__init__('velocity_tester')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        self.pwm_sub = self.create_subscription(
            String, '/motor_pwm', self._pwm_callback, 10)

        self._lock = threading.Lock()

        # Velocity recording
        self._t_vel      = []
        self._linear_meas  = []
        self._angular_meas = []

        # PWM recording
        self._t_pwm      = []
        self._pwm_left   = []
        self._pwm_right  = []

        self._recording  = False
        self._t_start    = 0.0

        self.get_logger().info("Velocity Tester ready.")

    def _odom_callback(self, msg):
        if not self._recording:
            return
        t = time.time() - self._t_start
        with self._lock:
            self._t_vel.append(t)
            self._linear_meas.append(msg.twist.twist.linear.x)
            self._angular_meas.append(msg.twist.twist.angular.z)

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

    def run_test(self, target_linear, target_angular, duration, ramp_time=0.0):
        print(f"\n🚀 Starting test: vx={target_linear:.3f} m/s | "
              f"wz={target_angular:.3f} rad/s | duration={duration}s")
        if ramp_time > 0:
            print(f"   Ramp time: {ramp_time}s")

        # Clear buffers
        with self._lock:
            self._t_vel      = []
            self._linear_meas  = []
            self._angular_meas = []
            self._t_pwm      = []
            self._pwm_left   = []
            self._pwm_right  = []

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
            t_vel      = list(self._t_vel)
            linear_meas  = list(self._linear_meas)
            angular_meas = list(self._angular_meas)
            t_pwm      = list(self._t_pwm)
            pwm_left   = list(self._pwm_left)
            pwm_right  = list(self._pwm_right)

        if len(t_vel) < 5:
            print("⚠️  Not enough odom data. Is /odom publishing?")
            return

        self._analyse_and_plot(
            t_vel, linear_meas, angular_meas,
            t_pwm, pwm_left, pwm_right,
            target_linear, target_angular, duration, ramp_time
        )

    def _send_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _analyse_and_plot(self, t_vel, linear_meas, angular_meas,
                           t_pwm, pwm_left, pwm_right,
                           target_linear, target_angular, duration, ramp_time):

        t_vel        = np.array(t_vel)
        linear_meas  = np.array(linear_meas)
        angular_meas = np.array(angular_meas)

        has_pwm = len(t_pwm) > 2
        if has_pwm:
            t_pwm     = np.array(t_pwm)
            pwm_left  = np.array(pwm_left)
            pwm_right = np.array(pwm_right)
        else:
            print("⚠️  No PWM data received. Is /motor_pwm being published?")

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

        # --- METRICS ---
        def analyse(meas, target, label):
            if abs(target) < 1e-6:
                return
            print(f"\n📊 {label} Analysis (target={target:.3f}):")
            ss_idx = int(len(meas) * 0.8)
            steady_state = np.mean(meas[ss_idx:])
            ss_error = abs(target - steady_state)
            print(f"   Steady-state value:  {steady_state:.4f}")
            print(f"   Steady-state error:  {ss_error:.4f}  ({ss_error/abs(target)*100:.1f}%)")
            idx_10 = next((i for i, v in enumerate(meas) if abs(v) >= 0.10 * abs(target)), None)
            idx_90 = next((i for i, v in enumerate(meas) if abs(v) >= 0.90 * abs(target)), None)
            if idx_10 is not None and idx_90 is not None:
                print(f"   Rise time (10→90%): {t_vel[idx_90] - t_vel[idx_10]:.3f}s")
            peak = np.max(np.abs(meas))
            overshoot = ((peak - abs(target)) / abs(target)) * 100
            print(f"   Peak value:         {peak:.4f}")
            if overshoot > 0:
                print(f"   Overshoot:          {overshoot:.1f}%")
            else:
                print(f"   Overshoot:          None")

        analyse(linear_meas,  target_linear,  "Linear  (vx)")
        analyse(angular_meas, target_angular, "Angular (wz)")

        if has_pwm:
            print(f"\n📊 PWM Summary:")
            print(f"   Left  — mean: {np.mean(pwm_left):.1f}  max: {np.max(np.abs(pwm_left)):.1f}")
            print(f"   Right — mean: {np.mean(pwm_right):.1f}  max: {np.max(np.abs(pwm_right)):.1f}")

        # --- PLOT ---
        BG   = '#0d1117'
        CARD = '#161b22'
        GRID = '#21262d'
        AXIS = '#8b949e'
        BLUE = '#58a6ff'
        GREEN= '#3fb950'
        RED  = '#f78166'
        PURP = '#d2a8ff'
        ORG  = '#ffa657'
        TEAL = '#39d353'

        nrows = 3 if has_pwm else 2
        fig = plt.figure(figsize=(14, 4 * nrows), facecolor=BG)
        fig.suptitle('Velocity Step Response — Closed Loop + PWM Analysis',
                     color='white', fontsize=13, fontweight='bold', y=0.99)

        gs = gridspec.GridSpec(nrows, 2, figure=fig, hspace=0.5, wspace=0.35)

        axes = []
        ax_lin  = fig.add_subplot(gs[0, :])   # Linear — full width
        ax_ang  = fig.add_subplot(gs[1, 0])   # Angular
        ax_err  = fig.add_subplot(gs[1, 1])   # Error
        axes += [ax_lin, ax_ang, ax_err]

        if has_pwm:
            ax_pwm_l = fig.add_subplot(gs[2, 0])  # Left PWM
            ax_pwm_r = fig.add_subplot(gs[2, 1])  # Right PWM
            axes += [ax_pwm_l, ax_pwm_r]

        for ax in axes:
            ax.set_facecolor(CARD)
            ax.tick_params(colors=AXIS, labelsize=8)
            for spine in ax.spines.values():
                spine.set_color(GRID)
            ax.yaxis.label.set_color(AXIS)
            ax.xaxis.label.set_color(AXIS)
            ax.title.set_color('white')
            ax.grid(True, color=GRID, linewidth=0.5, linestyle='--')

        # Linear velocity
        ax_lin.plot(t_vel, linear_target, color=BLUE, linewidth=1.5,
                    linestyle='--', label=f'Target ({target_linear:.3f} m/s)', alpha=0.8)
        ax_lin.plot(t_vel, linear_meas, color=GREEN, linewidth=1.5,
                    label='Measured', alpha=0.9)
        ax_lin.set_title('Linear Velocity (vx)')
        ax_lin.set_xlabel('Time (s)')
        ax_lin.set_ylabel('m/s')
        ax_lin.legend(loc='lower right', facecolor=CARD,
                      edgecolor=GRID, labelcolor='white', fontsize=8)

        # Angular velocity
        ax_ang.plot(t_vel, angular_target, color=RED, linewidth=1.5,
                    linestyle='--', label=f'Target ({target_angular:.3f} rad/s)', alpha=0.8)
        ax_ang.plot(t_vel, angular_meas, color=PURP, linewidth=1.5,
                    label='Measured', alpha=0.9)
        ax_ang.set_title('Angular Velocity (wz)')
        ax_ang.set_xlabel('Time (s)')
        ax_ang.set_ylabel('rad/s')
        ax_ang.legend(loc='lower right', facecolor=CARD,
                      edgecolor=GRID, labelcolor='white', fontsize=8)

        # Tracking error
        linear_error  = linear_target  - linear_meas
        angular_error = angular_target - angular_meas
        ax_err.plot(t_vel, linear_error,  color=BLUE, linewidth=1.2,
                    label='Linear error',  alpha=0.9)
        ax_err.plot(t_vel, angular_error, color=RED,  linewidth=1.2,
                    label='Angular error', alpha=0.9)
        ax_err.axhline(0, color=GRID, linewidth=0.8)
        ax_err.set_title('Tracking Error')
        ax_err.set_xlabel('Time (s)')
        ax_err.set_ylabel('Error')
        ax_err.legend(loc='upper right', facecolor=CARD,
                      edgecolor=GRID, labelcolor='white', fontsize=8)

        # PWM plots
        if has_pwm:
            ax_pwm_l.plot(t_pwm, pwm_left, color=ORG, linewidth=1.5,
                          label='Left PWM', alpha=0.9)
            ax_pwm_l.axhline(0, color=GRID, linewidth=0.8)
            ax_pwm_l.set_title('Left Wheel PWM')
            ax_pwm_l.set_xlabel('Time (s)')
            ax_pwm_l.set_ylabel('PWM (-255 to 255)')
            ax_pwm_l.set_ylim(-270, 270)
            ax_pwm_l.legend(loc='lower right', facecolor=CARD,
                            edgecolor=GRID, labelcolor='white', fontsize=8)

            ax_pwm_r.plot(t_pwm, pwm_right, color=TEAL, linewidth=1.5,
                          label='Right PWM', alpha=0.9)
            ax_pwm_r.axhline(0, color=GRID, linewidth=0.8)
            ax_pwm_r.set_title('Right Wheel PWM')
            ax_pwm_r.set_xlabel('Time (s)')
            ax_pwm_r.set_ylabel('PWM (-255 to 255)')
            ax_pwm_r.set_ylim(-270, 270)
            ax_pwm_r.legend(loc='lower right', facecolor=CARD,
                            edgecolor=GRID, labelcolor='white', fontsize=8)

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

    print("\n" + "="*55)
    print("  VELOCITY STEP RESPONSE TESTER  (+ PWM)")
    print("  Requires /odom, /cmd_vel, /motor_pwm")
    print("="*55)

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()