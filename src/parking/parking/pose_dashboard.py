#!/usr/bin/env python3
"""
pose_dashboard.py — Vision-Based Goal Pose Estimation Test Dashboard
=====================================================================
Combines a ROS 2 subscriber node with a Flask web server in a single
process using threading (no subprocesses, no zombies).

Workflow:
  1. ros2 run parking pose_dashboard
  2. Open http://<tailscale-ip>:5000 on laptop/phone
  3. Select Target Distance and Target Angle, position container
  4. Indicator turns GREEN when within tolerance → click Record
  5. After all runs, click "Finish & Generate Graphs"

Topics consumed:
  /image_rect       — camera stream (sensor_msgs/Image)
  /box_center_pose  — unified multi-tag bundle output (geometry_msgs/PoseStamped)

Hz is calculated as (messages received during the 30 s window) / 30,
which is exact and requires no timer or TF polling.

Output graphs saved to: ~/ros2_ws/src/parking/results/
"""

import os
import math
import time
import threading
import io
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import tf_transformations as tft

import cv2
from cv_bridge import CvBridge

import pandas as pd
import matplotlib
matplotlib.use('Agg')          # headless — no display needed
import matplotlib.pyplot as plt
import numpy as np

from flask import Flask, Response, render_template_string, jsonify, request

# ── Constants ─────────────────────────────────────────────────────────────────
IMAGE_TOPIC     = '/image_rect'
BOX_POSE_TOPIC  = '/box_center_pose'
OUTPUT_DIR      = os.path.expanduser('~/ros2_ws/src/parking/results/pose_estimation')
WEB_PORT      = 5000
STREAM_FPS    = 15
MARKER_TIMEOUT_S = 1.5   # marker considered lost after this many seconds
DIST_TOL_M    = 0.05
ANGLE_TOL_DEG = 2.0
RECORD_SECS   = 30
# ──────────────────────────────────────────────────────────────────────────────

# ── HTML Dashboard Template ────────────────────────────────────────────────────
_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Goal Pose Estimation Dashboard</title>
<style>
  :root {
    --bg: #0d1117; --card: #161b22; --border: #30363d;
    --text: #e6edf3; --muted: #8b949e;
    --green: #3fb950; --red: #f78166; --blue: #58a6ff;
    --yellow: #e3b341; --purple: #d2a8ff;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg); color: var(--text);
    font-family: 'Segoe UI', system-ui, sans-serif; font-size: 14px;
    padding: 16px;
  }
  h1 { font-size: 1.3rem; color: var(--blue); margin-bottom: 16px; }
  h2 { font-size: 0.95rem; color: var(--muted); text-transform: uppercase;
       letter-spacing: .08em; margin-bottom: 12px; }
  .grid {
    display: grid;
    grid-template-columns: 640px 1fr;
    gap: 16px; align-items: start;
  }
  .card {
    background: var(--card); border: 1px solid var(--border);
    border-radius: 8px; padding: 16px;
  }
  #feed {
    width: 100%; border-radius: 4px; display: block;
    background: #000; min-height: 200px;
  }
  .row { display: flex; gap: 12px; margin-bottom: 12px; align-items: center; }
  label { color: var(--muted); font-size: 12px; }
  select, input[type="number"] {
    background: var(--bg); color: var(--text);
    border: 1px solid var(--border); border-radius: 6px;
    padding: 6px 10px; font-size: 14px; flex: 1;
  }
  input[type="number"]::-webkit-inner-spin-button { opacity: 0.5; }
  /* ── Alignment Indicator ── */
  #indicator {
    border-radius: 8px; padding: 18px 12px; text-align: center;
    font-size: 1.1rem; font-weight: 700; letter-spacing: .05em;
    margin: 14px 0; transition: background 0.3s, color 0.3s;
  }
  #indicator.red  { background: rgba(247,129,102,.15); color: var(--red);
                    border: 2px solid var(--red); }
  #indicator.green { background: rgba(63,185,80,.15); color: var(--green);
                     border: 2px solid var(--green); }
  /* ── Live Readings ── */
  .readings {
    display: grid; grid-template-columns: 1fr 1fr;
    gap: 8px; margin-bottom: 14px;
  }
  .reading-cell {
    background: var(--bg); border-radius: 6px; padding: 10px;
    border: 1px solid var(--border);
  }
  .reading-label { color: var(--muted); font-size: 11px; margin-bottom: 4px; }
  .reading-value { font-size: 1.2rem; font-weight: 600; font-variant-numeric: tabular-nums; }
  .ok   { color: var(--green); }
  .warn { color: var(--red); }
  .na   { color: var(--muted); }
  /* ── Buttons ── */
  button {
    width: 100%; padding: 11px; border-radius: 6px; border: none;
    font-size: 14px; font-weight: 600; cursor: pointer;
    margin-bottom: 8px; transition: opacity .2s;
  }
  button:disabled { opacity: .4; cursor: not-allowed; }
  #btn_record { background: var(--blue); color: #000; }
  #btn_finish { background: var(--purple); color: #000; }
  /* ── Countdown ── */
  #countdown {
    text-align: center; font-size: 1.4rem; font-weight: 700;
    color: var(--yellow); min-height: 36px; margin-bottom: 8px;
    font-variant-numeric: tabular-nums;
  }
  #result_msg {
    margin-top: 8px; padding: 10px; border-radius: 6px;
    background: var(--bg); border: 1px solid var(--border);
    font-size: 12px; color: var(--muted); min-height: 40px;
    word-break: break-all;
  }
  .session-log {
    margin-top: 14px; font-size: 12px; color: var(--muted);
  }
  .session-log table {
    width: 100%; border-collapse: collapse; margin-top: 6px;
  }
  .session-log td, .session-log th {
    padding: 4px 8px; border-bottom: 1px solid var(--border); text-align: left;
  }
  .session-log th { color: var(--blue); font-size: 11px; }
  @media (max-width: 900px) {
    .grid { grid-template-columns: 1fr; }
  }
</style>
</head>
<body>
<h1>Vision-Based Goal Pose Estimation Dashboard</h1>
<div class="grid">

  <!-- LEFT: Camera Feed -->
  <div class="card">
    <h2>Camera Feed</h2>
    <img id="feed" src="/stream" alt="Camera stream">
  </div>

  <!-- RIGHT: Controls -->
  <div class="card">
    <h2>Test Configuration</h2>

    <div class="row">
      <label>Target Distance (m)</label>
      <input type="number" id="target_dist" value="1.5" min="0.1" max="10" step="0.1">
    </div>
    <div class="row">
      <label>Target Angle (°)</label>
      <input type="number" id="target_angle" value="0" min="-180" max="180" step="1">
    </div>

    <!-- Alignment Indicator -->
    <div id="indicator" class="red">NO MARKER DETECTED</div>

    <!-- Live Readings -->
    <div class="readings">
      <div class="reading-cell">
        <div class="reading-label">X Translation (m)</div>
        <div class="reading-value na" id="val_x">—</div>
      </div>
      <div class="reading-cell">
        <div class="reading-label">Y Offset (m)</div>
        <div class="reading-value na" id="val_y">—</div>
      </div>
      <div class="reading-cell">
        <div class="reading-label">Yaw (°)</div>
        <div class="reading-value na" id="val_yaw">—</div>
      </div>
      <div class="reading-cell">
        <div class="reading-label">Publish Hz</div>
        <div class="reading-value na" id="val_hz">—</div>
      </div>
    </div>

    <!-- Tolerance display -->
    <div class="readings" style="margin-bottom:14px">
      <div class="reading-cell">
        <div class="reading-label">Δ Distance</div>
        <div class="reading-value na" id="val_dx">—</div>
      </div>
      <div class="reading-cell">
        <div class="reading-label">Δ Angle</div>
        <div class="reading-value na" id="val_da">—</div>
      </div>
    </div>

    <!-- Record Button -->
    <button id="btn_record" disabled onclick="startRecording()">
      Start 30s Recording
    </button>
    <div id="countdown"></div>

    <!-- Finish Button -->
    <button id="btn_finish" onclick="finishAndGraph()">
      Finish &amp; Generate Graphs
    </button>

    <div id="result_msg">Waiting for data…</div>

    <!-- Session log -->
    <div class="session-log">
      <div>Recorded sessions: <strong id="session_count">0</strong>
           &nbsp;|&nbsp; Total samples: <strong id="sample_count">0</strong></div>
      <table>
        <thead>
          <tr><th>#</th><th>Dist (m)</th><th>Angle (°)</th><th>Samples</th><th>Mean X</th><th>σ X</th></tr>
        </thead>
        <tbody id="session_rows"></tbody>
      </table>
    </div>
  </div>
</div>

<script>
"use strict";
let _countdownId = null;
let _sessions    = [];

function getTargetDist()  { return parseFloat(document.getElementById('target_dist').value)  || 1.5; }
function getTargetAngle() { return parseFloat(document.getElementById('target_angle').value) || 0.0; }

function normalizeAngle(deg) {
  while (deg >  180) deg -= 360;
  while (deg < -180) deg += 360;
  return deg;
}

function pollState() {
  fetch('/state').then(r => r.json()).then(s => {
    const dist  = getTargetDist();
    const angle = getTargetAngle();

    // ── Live readings ──────────────────────────────────
    const fmtX = s.x !== null ? s.x.toFixed(3) : null;
    const fmtY = s.y !== null ? s.y.toFixed(3) : null;
    const fmtYaw = s.yaw_deg !== null ? s.yaw_deg.toFixed(1) : null;

    function setVal(id, val, ok) {
      const el = document.getElementById(id);
      el.textContent = val !== null ? val : '—';
      el.className   = 'reading-value ' + (val === null ? 'na' : (ok ? 'ok' : 'warn'));
    }

    // ── Tolerance deltas ──
    let distOk = false, angleOk = false;
    let dxStr = '—', daStr = '—';
    if (s.x !== null) {
      const dx = Math.abs(s.x - dist);
      dxStr = dx.toFixed(3) + ' m';
      distOk = dx <= {{ DIST_TOL_M }};
    }
    if (s.yaw_deg !== null) {
      const da = Math.abs(normalizeAngle(s.yaw_deg - angle));
      daStr = da.toFixed(1) + '°';
      angleOk = da <= {{ ANGLE_TOL_DEG }};
    }

    setVal('val_x',   fmtX,   distOk);
    setVal('val_y',   fmtY,   true);
    setVal('val_yaw', fmtYaw ? fmtYaw + '°' : null, angleOk);
    setVal('val_hz',  s.hz > 0 ? s.hz.toFixed(1) + ' Hz' : null, s.hz > 5);

    document.getElementById('val_dx').textContent = dxStr;
    document.getElementById('val_dx').className = 'reading-value ' + (s.x !== null ? (distOk ? 'ok' : 'warn') : 'na');
    document.getElementById('val_da').textContent = daStr;
    document.getElementById('val_da').className = 'reading-value ' + (s.yaw_deg !== null ? (angleOk ? 'ok' : 'warn') : 'na');

    // ── Alignment indicator ──
    const ind = document.getElementById('indicator');
    const aligned = s.marker_visible && distOk && angleOk;
    if (!s.marker_visible) {
      ind.className = 'red';
      ind.textContent = 'NO MARKER DETECTED';
    } else if (aligned) {
      ind.className = 'green';
      ind.textContent = '✓ ALIGNED — READY TO RECORD';
    } else {
      ind.className = 'red';
      const issues = [];
      if (!distOk)  issues.push('distance Δ=' + dxStr);
      if (!angleOk) issues.push('angle Δ=' + daStr);
      ind.textContent = 'OUT OF TOLERANCE: ' + issues.join(', ');
    }

    // ── Record button ──
    const btnRec = document.getElementById('btn_record');
    if (!s.recording && !_countdownId) {
      btnRec.disabled = !aligned;
    }

    // ── Counts ──
    document.getElementById('sample_count').textContent = s.samples_total;
    document.getElementById('session_count').textContent = s.sessions_count;

    // ── Update session table if new data ──
    if (s.sessions && s.sessions.length !== _sessions.length) {
      _sessions = s.sessions;
      const tbody = document.getElementById('session_rows');
      tbody.innerHTML = '';
      _sessions.forEach((sess, i) => {
        const tr = document.createElement('tr');
        tr.innerHTML = `<td>${i+1}</td><td>${sess.target_dist}</td>`
          + `<td>${sess.target_angle}</td><td>${sess.n}</td>`
          + `<td>${sess.mean_x.toFixed(3)}</td><td>${sess.std_x.toFixed(3)}</td>`;
        tbody.appendChild(tr);
      });
    }
  }).catch(() => {});
}

function startRecording() {
  const dist  = getTargetDist();
  const angle = getTargetAngle();
  fetch('/record', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ target_dist: dist, target_angle: angle })
  }).then(r => r.json()).then(data => {
    if (!data.ok) return;
    const btnRec = document.getElementById('btn_record');
    btnRec.disabled = true;
    const cdEl = document.getElementById('countdown');
    let remaining = {{ RECORD_SECS }};
    cdEl.textContent = `Recording… ${remaining}s`;
    _countdownId = setInterval(() => {
      remaining--;
      if (remaining > 0) {
        cdEl.textContent = `Recording… ${remaining}s`;
      } else {
        clearInterval(_countdownId);
        _countdownId = null;
        cdEl.textContent = 'Session complete ✓';
        btnRec.disabled = false;
        document.getElementById('result_msg').textContent =
          `Saved ${dist}m / ${angle}° session. Select next config and repeat.`;
      }
    }, 1000);
  });
}

function finishAndGraph() {
  const msg = document.getElementById('result_msg');
  msg.textContent = 'Generating graphs — please wait…';
  fetch('/finish', { method: 'POST' })
    .then(r => r.json())
    .then(data => { msg.textContent = data.message; });
}

setInterval(pollState, 400);
pollState();
</script>
</body>
</html>
"""


# ── Flask app (module-level) — references _node set in main() ────────────────
app  = Flask(__name__)
_node: 'PoseDashboardNode | None' = None


@app.route('/')
def index():
    html = _HTML.replace('{{ DIST_TOL_M }}',    str(DIST_TOL_M))
    html = html.replace('{{ ANGLE_TOL_DEG }}',  str(ANGLE_TOL_DEG))
    html = html.replace('{{ RECORD_SECS }}',    str(RECORD_SECS))
    return html


@app.route('/stream')
def stream():
    def generate():
        while True:
            frame = None
            if _node is not None:
                with _node._lock:
                    frame = _node._latest_frame
            if frame:
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n'
                    + frame
                    + b'\r\n'
                )
            time.sleep(1.0 / STREAM_FPS)

    return Response(
        generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/state')
def state():
    if _node is None:
        return jsonify({'error': 'node not ready'})
    return jsonify(_node.get_state())


@app.route('/record', methods=['POST'])
def record():
    if _node is None:
        return jsonify({'ok': False, 'error': 'node not ready'})
    data = request.get_json(force=True)
    ok   = _node.start_recording(
        float(data.get('target_dist',  1.5)),
        float(data.get('target_angle', 0.0))
    )
    return jsonify({'ok': ok})


@app.route('/finish', methods=['POST'])
def finish():
    if _node is None:
        return jsonify({'message': 'Node not ready.'})
    msg = _node.generate_graphs()
    return jsonify({'message': msg})


# ── ROS 2 Node ────────────────────────────────────────────────────────────────
class PoseDashboardNode(Node):

    def __init__(self):
        super().__init__('pose_dashboard')

        self._bridge = CvBridge()
        self._lock   = threading.Lock()

        # ── Shared image state ──────────────────────────────────────────────
        self._latest_frame: bytes | None = None

        # ── Shared pose state ───────────────────────────────────────────────
        self._current_x:       float | None = None   # forward (m)
        self._current_y:       float | None = None   # lateral (m)
        self._current_yaw_deg: float | None = None   # yaw from quaternion (°)
        self._last_seen:       float = 0.0            # time.time() of last message
        self._pose_times:      deque = deque(maxlen=60)  # rolling window for live Hz
        self._pose_hz:         float = 0.0            # rolling estimate shown in UI

        # ── Recording state ─────────────────────────────────────────────────
        self._recording:          bool  = False
        self._record_start:       float = 0.0
        self._record_target_dist: float = 0.0
        self._record_target_angle:float = 0.0
        self._record_rows:        list  = []   # current session rows

        # ── Accumulated sessions ────────────────────────────────────────────
        self._sessions:  list[dict] = []   # per-session summary dicts
        self._all_rows:  list[dict] = []   # all raw rows for graphing

        # ── QoS ─────────────────────────────────────────────────────────────
        be_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(Image, IMAGE_TOPIC, self._image_callback, be_qos)
        self.create_subscription(PoseStamped, BOX_POSE_TOPIC,
                                 self._box_pose_callback, be_qos)

        os.makedirs(OUTPUT_DIR, exist_ok=True)
        self.get_logger().info(
            f'Pose Dashboard ready — web: http://0.0.0.0:{WEB_PORT}'
        )

    # ── Image callback ────────────────────────────────────────────────────────
    def _image_callback(self, msg: Image):
        try:
            if msg.encoding in ('mono8', '8UC1'):
                cv_img = self._bridge.imgmsg_to_cv2(msg, 'mono8')
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
            else:
                cv_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}', throttle_duration_sec=5.0)
            return

        h, w = cv_img.shape[:2]
        target_w = 640
        if w != target_w:
            scale  = target_w / w
            cv_img = cv2.resize(cv_img, (target_w, int(h * scale)),
                                interpolation=cv2.INTER_LINEAR)

        ok, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 72])
        if ok:
            with self._lock:
                self._latest_frame = buf.tobytes()

    # ── Box pose callback ─────────────────────────────────────────────────────
    def _box_pose_callback(self, msg: PoseStamped):
        """
        Fired for every /box_center_pose message published by box_estimator.
        Pose is in odom frame: position.x/y are the box's map coordinates.
        """
        p = msg.pose.position
        o = msg.pose.orientation

        _, _, yaw_rad = tft.euler_from_quaternion([o.x, o.y, o.z, o.w])
        yaw_deg = math.degrees(yaw_rad)

        now = time.time()
        with self._lock:
            self._current_x       = float(p.x)
            self._current_y       = float(p.y)
            self._current_yaw_deg = yaw_deg
            self._last_seen       = now

            # Rolling Hz for live UI display only
            self._pose_times.append(now)
            if len(self._pose_times) >= 2:
                span = self._pose_times[-1] - self._pose_times[0]
                if span > 0:
                    self._pose_hz = (len(self._pose_times) - 1) / span

            if self._recording:
                elapsed = now - self._record_start
                if elapsed <= RECORD_SECS:
                    self._record_rows.append({
                        'timestamp':    now,
                        'elapsed_s':    elapsed,
                        'x':            float(p.x),
                        'y':            float(p.y),
                        'yaw_deg':      yaw_deg,
                        'target_dist':  self._record_target_dist,
                        'target_angle': self._record_target_angle,
                    })
                else:
                    self._finalise_session()

    # ── Session finalisation ──────────────────────────────────────────────────
    def _finalise_session(self):
        """Called with _lock held. Commits current session to summary + all_rows."""
        rows = self._record_rows
        self._recording    = False
        self._record_rows  = []

        if not rows:
            return

        xs   = [r['x']       for r in rows]
        yaws = [r['yaw_deg'] for r in rows]
        # Hz = exact message count over the fixed 30 s window.
        # This is the authoritative latency/load metric for Graph 3.
        summary = {
            'target_dist':  self._record_target_dist,
            'target_angle': self._record_target_angle,
            'n':            len(rows),
            'mean_x':       float(np.mean(xs)),
            'std_x':        float(np.std(xs)),
            'mean_yaw':     float(np.mean(yaws)),
            'std_yaw':      float(np.std(yaws)),
            'mean_hz':      float(len(rows) / RECORD_SECS),
        }
        self._sessions.append(summary)
        self._all_rows.extend(rows)

        self.get_logger().info(
            f'Session saved: dist={self._record_target_dist}m '
            f'angle={self._record_target_angle}° '
            f'n={summary["n"]} '
            f'mean_x={summary["mean_x"]:.3f} '
            f'hz={summary["mean_hz"]:.1f}'
        )

    # ── State accessor (Flask thread calls this) ──────────────────────────────
    def get_state(self) -> dict:
        with self._lock:
            age     = time.time() - self._last_seen
            visible = age < MARKER_TIMEOUT_S and self._current_x is not None
            return {
                'marker_visible': visible,
                'x':              self._current_x,
                'y':              self._current_y,
                'yaw_deg':        self._current_yaw_deg,
                'hz':             round(self._pose_hz, 1),
                'recording':      self._recording,
                'samples_total':  len(self._all_rows),
                'sessions_count': len(self._sessions),
                'sessions':       list(self._sessions),   # summary dicts
            }

    def start_recording(self, target_dist: float, target_angle: float) -> bool:
        with self._lock:
            if self._recording:
                return False
            self._recording           = True
            self._record_start        = time.time()
            self._record_target_dist  = target_dist
            self._record_target_angle = target_angle
            self._record_rows         = []
        self.get_logger().info(
            f'Recording started: {target_dist}m / {target_angle}°'
        )
        return True

    # ── Graph generation ──────────────────────────────────────────────────────
    def generate_graphs(self) -> str:
        with self._lock:
            all_rows  = list(self._all_rows)
            sessions  = list(self._sessions)

        if not all_rows:
            return 'No data collected yet — run at least one recording session.'

        df = pd.DataFrame(all_rows)
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        saved = []

        # ── Graph 1: Depth Accuracy ─────────────────────────────────────────
        # X-axis: target distance (at 0° angle only)
        # Y-axis: |mean(estimated X) − target_dist|
        df0 = df[df['target_angle'] == 0.0]
        if not df0.empty:
            grp   = df0.groupby('target_dist')['x']
            dists = sorted(grp.groups.keys())
            errs  = [abs(grp.get_group(d).mean() - d) for d in dists]

            fig, ax = plt.subplots(figsize=(8, 5))
            bars = ax.bar([str(d) for d in dists], errs, color='steelblue',
                          edgecolor='white', linewidth=0.5)
            ax.bar_label(bars, fmt='%.3f m', padding=3, fontsize=9)
            ax.set_xlabel('Target Distance (m)')
            ax.set_ylabel('Mean Absolute Depth Error (m)')
            ax.set_title('Graph 1 — Depth Accuracy (0° approach angle)')
            ax.grid(axis='y', alpha=0.35, linestyle='--')
            ax.set_ylim(0, max(errs) * 1.3 + 0.01)
            fig.tight_layout()
            p = os.path.join(OUTPUT_DIR, 'graph1_depth_accuracy.png')
            fig.savefig(p, dpi=150); plt.close(fig); saved.append(p)
        else:
            saved.append('(Graph 1 skipped — no 0° sessions)')

        # ── Graph 2: Spatial Jitter ─────────────────────────────────────────
        # X-axis: target angle (at 1.5 m only)
        # Y-axis: σ for X, Y, and Yaw separately
        df15 = df[df['target_dist'] == 1.5]
        if not df15.empty:
            grp    = df15.groupby('target_angle')
            angles = sorted(grp.groups.keys())
            sx  = [grp.get_group(a)['x'].std()       for a in angles]
            sy  = [grp.get_group(a)['y'].std()       for a in angles]
            sw  = [grp.get_group(a)['yaw_deg'].std() for a in angles]

            x_pos  = np.arange(len(angles))
            width  = 0.25
            fig, ax = plt.subplots(figsize=(9, 5))
            ax.bar(x_pos - width, sx, width, label='σ X (m)',   color='steelblue')
            ax.bar(x_pos,         sy, width, label='σ Y (m)',   color='coral')
            ax.bar(x_pos + width, sw, width, label='σ Yaw (°)', color='mediumseagreen')
            ax.set_xticks(x_pos)
            ax.set_xticklabels([f'{int(a)}°' for a in angles])
            ax.set_xlabel('Target Angle (°)')
            ax.set_ylabel('Standard Deviation')
            ax.set_title('Graph 2 — Spatial Jitter @ 1.5 m')
            ax.legend()
            ax.grid(axis='y', alpha=0.35, linestyle='--')
            fig.tight_layout()
            p = os.path.join(OUTPUT_DIR, 'graph2_spatial_jitter.png')
            fig.savefig(p, dpi=150); plt.close(fig); saved.append(p)
        else:
            saved.append('(Graph 2 skipped — no 1.5 m sessions)')

        # ── Graph 3: Publish Frequency vs Angle ────────────────────────────
        # X-axis: target angle; Y-axis: average Hz during session
        if sessions:
            sess_df = pd.DataFrame(sessions)
            grp     = sess_df.groupby('target_angle')['mean_hz'].mean()
            angles  = sorted(grp.index.tolist())
            hz_vals = [grp[a] for a in angles]

            fig, ax = plt.subplots(figsize=(8, 5))
            ax.plot([f'{int(a)}°' for a in angles], hz_vals,
                    marker='o', color='mediumpurple', linewidth=2, markersize=7)
            for xi, yi in zip(range(len(angles)), hz_vals):
                ax.annotate(f'{yi:.1f}', (xi, yi),
                            textcoords='offset points', xytext=(0, 8),
                            ha='center', fontsize=9)
            ax.set_xlabel('Target Angle (°)')
            ax.set_ylabel('Average Publish Frequency (Hz)')
            ax.set_title('Graph 3 — /box_center_pose Publish Frequency vs Approach Angle')
            ax.grid(alpha=0.35, linestyle='--')
            ax.set_ylim(0, max(hz_vals) * 1.25 + 1)
            fig.tight_layout()
            p = os.path.join(OUTPUT_DIR, 'graph3_frequency.png')
            fig.savefig(p, dpi=150); plt.close(fig); saved.append(p)

        # ── Save raw CSV ────────────────────────────────────────────────────
        ts      = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_out = os.path.join(OUTPUT_DIR, f'raw_data_{ts}.csv')
        df.to_csv(csv_out, index=False)
        saved.append(csv_out)

        self.get_logger().info(f'Graphs saved to {OUTPUT_DIR}')
        return (
            f'Saved to {OUTPUT_DIR}: '
            + ', '.join(os.path.basename(str(p)) for p in saved)
        )


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    global _node
    rclpy.init(args=args)
    _node = PoseDashboardNode()

    # Flask runs in a daemon thread — dies automatically when rclpy.spin() exits
    flask_thread = threading.Thread(
        target=lambda: app.run(
            host='0.0.0.0',
            port=WEB_PORT,
            threaded=True,
            use_reloader=False,   # reloader spawns a child process — avoid it
            debug=False,
        ),
        daemon=True,
        name='flask-dashboard',
    )
    flask_thread.start()

    try:
        rclpy.spin(_node)
    except KeyboardInterrupt:
        pass
    finally:
        _node.destroy_node()
        rclpy.shutdown()
        # flask_thread is daemon — it dies here with no zombies


if __name__ == '__main__':
    main()
