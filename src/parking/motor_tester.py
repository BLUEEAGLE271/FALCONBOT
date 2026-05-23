#!/usr/bin/env python3
"""
motor_tester.py — Standalone ESP32 Motor Control Test Sequencer
================================================================
Sends velocity profiles over serial, records telemetry on a
background daemon thread, and saves CSV + PNG results automatically.

Usage:
    python3 motor_tester.py [/dev/esp32]   (port defaults to /dev/esp32)

Serial protocol:
    TX to ESP32 — velocity command (CMD_ROS_CTRL):
        {"T":13,"X":<linear_x>,"Z":<angular_z>}\n

    TX to ESP32 — live PID update (CMD_SET_MOTOR_PID):
        {"T":1231,"P":<p>,"I":<i>,"D":<d>,"L":255}\n

    RX from ESP32 — motor telemetry lines (high-frequency, prefixed):
        T,LeftTarget,LeftActual,RightTarget,RightActual\n

    RX from ESP32 — IMU / other JSON lines (prefixed with '{'):
        {"roll":...,"pitch":...}   ← silently ignored
"""

import os
import sys
import time
import threading
from datetime import datetime

import serial
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')    # headless — no GUI
import matplotlib.pyplot as plt

# ── Configuration ──────────────────────────────────────────────────────────────
SERIAL_PORT  = '/dev/esp32'
BAUD_RATE    = 115200
RESULTS_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')
# ──────────────────────────────────────────────────────────────────────────────

# ── Shared telemetry state (protected by _lock) ────────────────────────────────
_ser:       serial.Serial | None = None
_lock       = threading.Lock()
_data:      list[dict]           = []
_recording: bool                 = False
_t0:        float                = 0.0           # monotonic reference for this run
# ──────────────────────────────────────────────────────────────────────────────


# ─────────────────────────────────────────────────────────────────────────────
# SERIAL READER THREAD
# Runs as a daemon so it dies automatically when the main thread exits.
# Never touches _recording or _t0 without the lock.
# ─────────────────────────────────────────────────────────────────────────────

def _reader_thread() -> None:
    while True:
        try:
            raw = _ser.readline()
        except serial.SerialException:
            time.sleep(0.01)
            continue

        line = raw.decode('ascii', errors='ignore').strip()
        if not line:
            continue

        # Only process motor telemetry lines — everything else (IMU JSON, etc.) is ignored.
        if not line.startswith('T,'):
            continue

        # Strip the "T," prefix and parse the 4 telemetry floats.
        parts = line[2:].split(',')
        if len(parts) < 4:
            continue

        try:
            row = {
                't':            time.monotonic() - _t0,
                'left_target':  float(parts[0]),
                'left_actual':  float(parts[1]),
                'right_target': float(parts[2]),
                'right_actual': float(parts[3]),
            }
            if len(parts) >= 6:
                row['left_pwm']  = float(parts[4])
                row['right_pwm'] = float(parts[5])
        except ValueError:
            continue

        with _lock:
            if _recording:
                _data.append(row)


# ─────────────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def _send_velocity(linear_x: float, angular_z: float) -> None:
    """Send CMD_ROS_CTRL — ESP32 converts X/Z to per-wheel PWM internally."""
    cmd = f'{{"T":13,"X":{linear_x:.3f},"Z":{angular_z:.3f}}}\n'
    _ser.write(cmd.encode('ascii'))


def _hold_velocity(linear_x: float, angular_z: float, duration: float, interval: float = 0.2) -> None:
    """Re-send the velocity command every `interval` seconds for `duration` seconds.
    Prevents the ESP32 safety timeout from zeroing the target mid-test."""
    end = time.monotonic() + duration
    while True:
        _send_velocity(linear_x, angular_z)
        remaining = end - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(interval, remaining))


def _send_pid(p: float, i: float, d: float) -> None:
    """Send CMD_SET_MOTOR_PID — live PID update, L=255 targets both wheels."""
    cmd = f'{{"T":1231,"P":{p},"I":{i},"D":{d},"L":255}}\n'
    _ser.write(cmd.encode('ascii'))


def _start_recording() -> None:
    global _recording, _t0, _data
    with _lock:
        _data      = []
        _t0        = time.monotonic()
        _recording = True


def _stop_recording() -> list[dict]:
    global _recording
    with _lock:
        _recording = False
        return list(_data)


# ─────────────────────────────────────────────────────────────────────────────
# TEST SEQUENCES  (time.sleep only — reader runs concurrently on its own thread)
# ─────────────────────────────────────────────────────────────────────────────

def test_step_response() -> tuple[list, str]:
    """Step response test at 0.5 m/s (Z=0.0)."""
    print('\n[TEST 1] Step Response')
    print('         X: 0.0 → 0.50 m/s → 0.0   Z: 0.0')
    _start_recording()

    _hold_velocity(0.0, 0.0, 1.0)
    _hold_velocity(0.5, 0.0, 4.0)
    _hold_velocity(0.0, 0.0, 1.0)

    rows = _stop_recording()
    print(f'  Captured {len(rows)} samples.')
    return rows, 'step_response'


def test_zero_radius_turn() -> tuple[list, str]:
    """X=0.0, user-chosen Z rad/s for 3 s."""
    print('\n[TEST 2] Zero-Radius Turn')
    try:
        z_str = input('  Angular velocity Z (rad/s) [default 1.5]: ').strip()
        z_val = float(z_str) if z_str else 1.5
    except ValueError:
        print('  Invalid input — using 1.5 rad/s.')
        z_val = 1.5

    print(f'         X: 0.0   Z: 0.0 → {z_val} rad/s → 0.0')
    _start_recording()

    _hold_velocity(0.0, 0.0, 1.0)
    _hold_velocity(0.0, z_val, 3.0)
    _hold_velocity(0.0, 0.0, 1.0)

    rows = _stop_recording()
    print(f'  Captured {len(rows)} samples.')
    return rows, 'zero_radius_turn'


def test_ramp_profile() -> tuple[list, str]:
    """Two identical ramp cycles (Z=0.0 throughout):
      t 0–1 s : idle at 0.0
      t 1–2 s : ramp up   0.1 → 0.5  (5 steps × 200 ms)
      t 2–3 s : hold at 0.5
      t 3–4 s : ramp down 0.4 → 0.0  (5 steps × 200 ms)
      t 4–5 s : ramp up   (repeat)
      t 5–6 s : hold
      t 6–7 s : ramp down (repeat)
      t 7–8 s : idle at 0.0
    """
    # 5 steps × 0.1 m/s × 200 ms = exactly 1.0 s per ramp edge.
    STEPS_UP   = [round(0.1 * n, 10) for n in range(1, 6)]   # 0.1 … 0.5
    STEPS_DOWN = list(reversed(STEPS_UP[:-1])) + [0.0]        # 0.4 … 0.0

    print('\n[TEST 3] Double Ramp Profile')
    print(f'         Up:   {[f"{v:.1f}" for v in STEPS_UP]}  (×200 ms = 1 s)')
    print(f'         Hold: 0.5 m/s for 1 s  ×2 cycles')
    print(f'         Down: {[f"{v:.1f}" for v in STEPS_DOWN]}  (×200 ms = 1 s)')
    _start_recording()

    _hold_velocity(0.0, 0.0, 1.0)          # t 0–1  idle

    for _ in range(2):                      # two identical cycles
        for v in STEPS_UP:                  # ramp up   (1 s)
            _hold_velocity(v, 0.0, 0.2)
        _hold_velocity(0.5, 0.0, 1.0)      # hold      (1 s)
        for v in STEPS_DOWN:               # ramp down (1 s)
            _hold_velocity(v, 0.0, 0.2)

    _hold_velocity(0.0, 0.0, 1.0)          # t 7–8  idle

    rows = _stop_recording()
    print(f'  Captured {len(rows)} samples.')
    return rows, 'ramp_profile'


# ─────────────────────────────────────────────────────────────────────────────
# STATISTICS
# ─────────────────────────────────────────────────────────────────────────────

def _rmse(target: np.ndarray, actual: np.ndarray) -> float:
    return float(np.sqrt(np.mean((target - actual) ** 2)))


def _step_stats(df: pd.DataFrame) -> dict:
    """
    Max overshoot and settling time (±2 % band) for each wheel.

    The ESP32 converts X=0.5 to its own wheel-speed target, so we read the
    steady-state target directly from the telemetry: median of each wheel's
    reported target over the latter half of the step window (t ∈ [2.5, 4.0)).
    This is robust against the ramp-up transient at the start of the step.

    Settling time is measured from step application (t ≈ 1.0 s).
    """
    T_STEP      = 1.0    # nominal step-apply time
    T_STEP_END  = 5.0    # nominal step-end time
    BAND_PCT    = 0.02   # ±2 % of target

    step_mask   = (df['t'] >= T_STEP)  & (df['t'] < T_STEP_END)
    steady_mask = (df['t'] >= 3.5)     & (df['t'] < T_STEP_END)
    step_df     = df[step_mask]

    result = {}
    for side in ('left', 'right'):
        actual = step_df[f'{side}_actual'].to_numpy()
        t_arr  = step_df['t'].to_numpy()

        # Derive per-wheel target from the ESP32's own reported target value.
        target_col = df[steady_mask][f'{side}_target']
        target     = float(target_col.median()) if len(target_col) else 0.0
        band       = BAND_PCT * abs(target) if target else 0.01

        peak      = float(actual.max()) if len(actual) else 0.0
        overshoot = max(0.0, peak - target)
        over_pct  = (overshoot / target * 100.0) if target else 0.0

        # Scan backward: last sample outside ±band is the settling boundary.
        in_band  = np.abs(actual - target) <= band
        settle_t = None
        for i in range(len(in_band) - 1, -1, -1):
            if not in_band[i]:
                idx = i + 1
                if idx < len(t_arr):
                    settle_t = float(t_arr[idx]) - T_STEP
                break
        else:
            if len(t_arr):
                settle_t = float(t_arr[0]) - T_STEP

        result[side] = {
            'target':   target,
            'peak':     peak,
            'over_m_s': overshoot,
            'over_pct': over_pct,
            'settle_s': settle_t,
        }
    return result


# ─────────────────────────────────────────────────────────────────────────────
# SAVE + PLOT
# ─────────────────────────────────────────────────────────────────────────────

# Dark-theme palette matching the rest of the project
_BG    = '#0d1117'
_CARD  = '#161b22'
_GRID  = '#21262d'
_MUTED = '#8b949e'
_BLUE  = '#58a6ff'
_GREEN = '#3fb950'
_RED   = '#f78166'
_YELL  = '#e3b341'


def _style_ax(ax) -> None:
    ax.set_facecolor(_CARD)
    ax.grid(True, color=_GRID, linewidth=0.6, linestyle='--')
    for spine in ax.spines.values():
        spine.set_color(_GRID)
    ax.tick_params(colors=_MUTED, labelsize=8)
    ax.xaxis.label.set_color(_MUTED)
    ax.yaxis.label.set_color(_MUTED)
    ax.title.set_color('white')


def _ideal_target(t_arr: np.ndarray, target_arr: np.ndarray, test_name: str) -> np.ndarray:
    """Smooth piecewise-linear reference for plotting.
    Ramp profile: replaces discrete step staircase with clean diagonals.
    All other tests: returns target_arr unchanged."""
    if test_name != 'ramp_profile':
        return target_arr
    peak = float(np.max(np.abs(target_arr)))
    t_ref = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
    v_ref = [0.0, 0.0, peak, peak, 0.0, 0.0, peak, peak, 0.0]
    return np.interp(t_arr, t_ref, v_ref)


def save_and_plot(rows: list[dict], test_name: str) -> None:
    if not rows:
        print('  No data — skipping save.')
        return

    os.makedirs(RESULTS_DIR, exist_ok=True)
    ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
    stem     = f'{test_name}_{ts}'
    csv_path = os.path.join(RESULTS_DIR, stem + '.csv')
    png_path = os.path.join(RESULTS_DIR, stem + '.png')

    df = pd.DataFrame(rows)
    df.to_csv(csv_path, index=False)
    print(f'  CSV → {csv_path}')

    # ── Build stats string ────────────────────────────────────────────────────
    if test_name == 'step_response':
        s = _step_stats(df)
        lines = []
        for side in ('left', 'right'):
            st = s[side]
            settle_str = (f"{st['settle_s']:.3f} s"
                          if st['settle_s'] is not None else 'n/a')
            lines.append(
                f"{side.capitalize():5s} │ Target {st['target']:.3f} m/s │ "
                f"Peak {st['peak']:.3f} m/s │ "
                f"Overshoot {st['over_pct']:.1f}% ({st['over_m_s']:.4f} m/s) │ "
                f"Settling time {settle_str}"
            )
        stats_str = '\n'.join(lines)
    else:
        rmse_l = _rmse(df['left_target'].to_numpy(),  df['left_actual'].to_numpy())
        rmse_r = _rmse(df['right_target'].to_numpy(), df['right_actual'].to_numpy())
        stats_str = (f"RMSE  │  Left: {rmse_l:.5f} m/s  │  Right: {rmse_r:.5f} m/s")

    print(f'  Stats: {stats_str.replace(chr(10), "  //  ")}')

    # ── Figure ────────────────────────────────────────────────────────────────
    fig, (ax_l, ax_r) = plt.subplots(
        2, 1, figsize=(12, 7), sharex=True,
        gridspec_kw={'hspace': 0.35}
    )
    fig.patch.set_facecolor(_BG)
    fig.suptitle(
        f'{test_name.replace("_", " ").title()}   [{ts}]',
        color='white', fontsize=13, fontweight='bold', y=0.98
    )

    plot_specs = [
        (ax_l, 'left',  _BLUE,  _GREEN, 'Left Wheel'),
        (ax_r, 'right', _RED,   _YELL,  'Right Wheel'),
    ]
    for ax, side, c_target, c_actual, label in plot_specs:
        t_smooth = _ideal_target(df['t'].to_numpy(), df[f'{side}_target'].to_numpy(), test_name)
        ax.plot(df['t'], t_smooth,
                color=c_target, linewidth=1.8, linestyle='--',
                label='Target', alpha=0.9)
        ax.plot(df['t'], df[f'{side}_actual'],
                color=c_actual, linewidth=1.4,
                label='Actual', alpha=0.95)
        ax.set_title(label, fontsize=10, pad=4)
        ax.set_ylabel('Velocity (m/s)')
        ax.legend(loc='upper right', fontsize=8,
                  facecolor=_CARD, edgecolor=_GRID, labelcolor='white')
        _style_ax(ax)

    ax_r.set_xlabel('Time (s)')

    # Stats annotation at the bottom of the figure
    fig.text(
        0.5, 0.01, stats_str,
        ha='center', va='bottom', fontsize=8.5, color=_MUTED,
        family='monospace',
        bbox=dict(boxstyle='round,pad=0.4', facecolor=_CARD, edgecolor=_GRID, alpha=0.9)
    )

    fig.tight_layout(rect=[0, 0.08, 1, 0.96])
    fig.savefig(png_path, dpi=150, bbox_inches='tight', facecolor=_BG)
    plt.close(fig)
    print(f'  PNG  → {png_path}')


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def menu_pid_update() -> None:
    """Prompt for P/I/D values and send CMD_SET_MOTOR_PID to the ESP32."""
    print('\n[PID] Enter new gains (leave blank to cancel):')
    try:
        p_str = input('  P gain: ').strip()
        if not p_str:
            print('  Cancelled.')
            return
        i_str = input('  I gain: ').strip()
        d_str = input('  D gain: ').strip()
        p = float(p_str)
        i = float(i_str)
        d = float(d_str)
    except ValueError:
        print('  Invalid input — PID not updated.')
        return

    _send_pid(p, i, d)
    print(f'  Sent → {{"T":1231,"P":{p},"I":{i},"D":{d},"L":255}}')


_MENU = """
┌─────────────────────────────────────┐
│  Motor Test Sequencer  [{port}]
├─────────────────────────────────────┤
│  1 — Step Response                  │
│  2 — Zero-Radius Turn               │
│  3 — Ramp Profile                   │
│  4 — Update PID Gains               │
│  q — Quit                           │
└─────────────────────────────────────┘"""


def main() -> None:
    global _ser

    port = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT
    print(f'Opening {port} @ {BAUD_RATE} baud…')

    try:
        _ser = serial.Serial(port, BAUD_RATE, timeout=1.0)
    except serial.SerialException as exc:
        print(f'ERROR: Cannot open {port}: {exc}')
        sys.exit(1)

    # Give the ESP32 time to reboot after the DTR-reset triggered by pyserial
    time.sleep(2.0)

    reader = threading.Thread(target=_reader_thread, daemon=True, name='serial-reader')
    reader.start()
    print('Reader thread running.')

    TESTS = {
        '1': test_step_response,
        '2': test_zero_radius_turn,
        '3': test_ramp_profile,
    }

    try:
        while True:
            print(_MENU.format(port=port))
            choice = input('Select: ').strip().lower()

            if choice in TESTS:
                rows, name = TESTS[choice]()
                save_and_plot(rows, name)
            elif choice == '4':
                menu_pid_update()
            elif choice == 'q':
                print('Stopping motors…')
                _send_velocity(0.0, 0.0)
                break
            else:
                print('  Unknown option.')
    except KeyboardInterrupt:
        print('\nInterrupted — stopping motors.')
        _send_velocity(0.0, 0.0)
    finally:
        _ser.close()


if __name__ == '__main__':
    main()
