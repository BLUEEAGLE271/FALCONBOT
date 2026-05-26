#!/usr/bin/env python3
"""
Interactive camera capture for AprilTag pipeline visualisation.

Press Enter → saves 4 images to results/images/. Ctrl+C to quit.

NOTE: Stop mission.launch.py before running — both use nvarguscamerasrc
      and the camera cannot be shared between two processes.

Output per capture (results/images/shotNN_TIMESTAMP_*.png):
  1_distorted_1.25x     — full res, fisheye distortion exaggerated ×1.25
  2_undistorted         — full res, fisheye corrected
  3_nvidia_gray_820x616 — 820×616 grayscale (Isaac ROS AprilTag input size)
  4_apriltag_axes       — image 3 + thick colour-coded X/Y/Z axes on each tag
"""

import os
import sys
import cv2
import numpy as np
from datetime import datetime

# ── Paths ────────────────────────────────────────────────────────────────
_HERE       = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.normpath(os.path.join(_HERE, '..', 'results', 'images'))

# ── Camera intrinsics — full resolution 3280×2464 ────────────────────────
K = np.array([
    [906.5,   0.,    817.08],
    [  0.,  913.31,  629.30],
    [  0.,    0.,      1.  ],
], dtype=np.float64)

# equidistant fisheye (k1 k2 k3 k4)
D = np.array([-0.022401, -0.022708, 0.014203, -0.009313], dtype=np.float64)

FULL_W, FULL_H = 3280, 2464

# ── Nvidia ROS2 pipeline size (isaac_ros_image_proc rectify output) ──────
NVIDIA_W, NVIDIA_H = 820, 616

# ── AprilTag detector — same family as the mission stack ─────────────────
_DICT   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
_PARAMS = cv2.aruco.DetectorParameters()
DETECTOR = cv2.aruco.ArucoDetector(_DICT, _PARAMS)
TAG_SIZE = 0.103  # metres

# ── Scaled camera matrix for 820×616 undistorted image ───────────────────
_sx = NVIDIA_W / FULL_W   # 0.25
_sy = NVIDIA_H / FULL_H   # 0.25
K_SMALL = np.array([
    [K[0, 0] * _sx, 0.,            K[0, 2] * _sx],
    [0.,            K[1, 1] * _sy, K[1, 2] * _sy],
    [0.,            0.,             1.            ],
], dtype=np.float64)


# ── Precompute remap maps (slow once, fast per capture) ──────────────────

def _make_undistort_maps():
    """Maps that remove fisheye distortion at full resolution."""
    m1, m2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), K, (FULL_W, FULL_H), cv2.CV_32FC1
    )
    return m1, m2


def _make_distort_maps(scale: float = 1.25):
    """
    Forward-distortion maps: apply fisheye with D*scale to an undistorted image.

    For each output pixel treated as a distorted coordinate, undistortPoints
    finds the corresponding undistorted source position. remap then samples
    the undistorted image there, producing a (more) distorted output.
    """
    D_scaled = D * scale
    h, w = FULL_H, FULL_W

    xx, yy = np.meshgrid(
        np.arange(w, dtype=np.float32),
        np.arange(h, dtype=np.float32),
    )
    pts = np.stack([xx.ravel(), yy.ravel()], axis=1).reshape(-1, 1, 2)
    src = cv2.fisheye.undistortPoints(pts, K, D_scaled, P=K)
    src = src.reshape(h, w, 2)
    return src[..., 0].copy(), src[..., 1].copy()


def _precompute():
    print("Precomputing distortion maps (one-time, ~5 s)…")
    u1, u2 = _make_undistort_maps()
    print("  undistort map done")
    d1, d2 = _make_distort_maps(scale=1.25)
    print("  forward-distort ×1.25 map done")
    return u1, u2, d1, d2


# ── GStreamer pipeline ────────────────────────────────────────────────────

def _gst_pipeline() -> str:
    return (
        f"nvarguscamerasrc sensor-id=0 "
        f"exposuretimerange='13000 60000' gainrange='1 2' ! "
        f"video/x-raw(memory:NVMM), width={FULL_W}, height={FULL_H}, framerate=21/1 ! "
        f"nvvidconv compute-hw=1 ! "
        f"video/x-raw(memory:NVMM), width={FULL_W}, height={FULL_H}, format=NV12 ! "
        f"nvvidconv compute-hw=1 ! "
        f"video/x-raw, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! "
        f"appsink max-buffers=2 drop=true"
    )


# ── AprilTag axis drawing ─────────────────────────────────────────────────

def _draw_axes(img_bgr: np.ndarray, corners: np.ndarray, tag_id: int) -> np.ndarray:
    """
    Draw thick X/Y/Z axes on one detected AprilTag via solvePnP.
    corners: (4, 2) float32, clockwise from top-left in the 820×616 image.
    """
    half = TAG_SIZE / 2.0
    obj_pts = np.array([
        [-half,  half, 0.],
        [ half,  half, 0.],
        [ half, -half, 0.],
        [-half, -half, 0.],
    ], dtype=np.float64)

    img_pts   = corners.reshape(4, 2).astype(np.float64)
    dist_zero = np.zeros(4)

    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K_SMALL, dist_zero)
    if not ok:
        return img_bgr

    axis_len = TAG_SIZE * 1.3

    # drawFrameAxes: X=red, Y=green, Z=blue — thickness 5 for visibility
    cv2.drawFrameAxes(img_bgr, K_SMALL, dist_zero, rvec, tvec,
                      axis_len, thickness=5)

    # Bold X Y Z labels with black outline for readability
    tips_3d = np.float32([
        [axis_len, 0, 0],
        [0, axis_len, 0],
        [0, 0, axis_len],
    ]).reshape(-1, 1, 3)
    tips, _ = cv2.projectPoints(tips_3d, rvec, tvec, K_SMALL, dist_zero)
    tips = tips.reshape(-1, 2).astype(int)

    font = cv2.FONT_HERSHEY_SIMPLEX
    for label, pt, fg in [
        ('X', tips[0], (0,   0, 255)),   # red
        ('Y', tips[1], (0, 255,   0)),   # green
        ('Z', tips[2], (255,  0,   0)),  # blue
    ]:
        cv2.putText(img_bgr, label, tuple(pt), font, 1.4, (0, 0, 0), 7)
        cv2.putText(img_bgr, label, tuple(pt), font, 1.4, fg,       3)

    # Tag ID at centre
    cx, cy = corners.mean(axis=0).astype(int)
    cv2.putText(img_bgr, f'ID:{tag_id}', (cx, cy), font, 0.8, (0, 0, 0),    4)
    cv2.putText(img_bgr, f'ID:{tag_id}', (cx, cy), font, 0.8, (255, 255, 0), 2)

    return img_bgr


# ── Main capture + save ──────────────────────────────────────────────────

def save_images(raw: np.ndarray,
                shot_id: str,
                umap1, umap2,
                dmap1, dmap2) -> None:

    os.makedirs(RESULTS_DIR, exist_ok=True)
    prefix = os.path.join(RESULTS_DIR, shot_id)

    # ── Image 2: full res, undistorted ───────────────────────────────────
    img2 = cv2.remap(raw, umap1, umap2, cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT)
    p2 = f"{prefix}_2_undistorted.png"
    cv2.imwrite(p2, img2)
    print(f"  [2] saved → {os.path.basename(p2)}")

    # ── Image 1: full res, distortion ×1.25 applied ──────────────────────
    img1 = cv2.remap(img2, dmap1, dmap2, cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT)
    p1 = f"{prefix}_1_distorted_1.25x.png"
    cv2.imwrite(p1, img1)
    print(f"  [1] saved → {os.path.basename(p1)}")

    # ── Image 3: 820×616 grayscale (Isaac ROS AprilTag input size) ───────
    img3_color = cv2.resize(img2, (NVIDIA_W, NVIDIA_H), interpolation=cv2.INTER_AREA)
    img3 = cv2.cvtColor(img3_color, cv2.COLOR_BGR2GRAY)
    p3 = f"{prefix}_3_nvidia_gray_{NVIDIA_W}x{NVIDIA_H}.png"
    cv2.imwrite(p3, img3)
    print(f"  [3] saved → {os.path.basename(p3)}")

    # ── Image 4: image 3 + AprilTag X/Y/Z axes ───────────────────────────
    img4 = cv2.cvtColor(img3, cv2.COLOR_GRAY2BGR)
    corners, ids, _ = DETECTOR.detectMarkers(img3)

    if ids is not None and len(ids) > 0:
        for corner, tag_id in zip(corners, ids.flatten()):
            img4 = _draw_axes(img4, corner[0], int(tag_id))
        print(f"  [4] detected {len(ids)} tag(s): {ids.flatten().tolist()}")
    else:
        cv2.putText(img4, "No AprilTag detected", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        print("  [4] no AprilTags detected")

    p4 = f"{prefix}_4_apriltag_axes.png"
    cv2.imwrite(p4, img4)
    print(f"  [4] saved → {os.path.basename(p4)}")


def main() -> None:
    print("=" * 60)
    print("  Image Capture — AprilTag pipeline verification")
    print("=" * 60)
    print()
    print("IMPORTANT: mission.launch.py must NOT be running.")
    print()

    umap1, umap2, dmap1, dmap2 = _precompute()
    print()

    print("Opening camera…")
    cap = cv2.VideoCapture(_gst_pipeline(), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("ERROR: Camera failed to open.")
        print("  → Is mission.launch.py still running? (nvargus conflict)")
        sys.exit(1)

    print("Warming up (draining 10 frames for exposure settle)…")
    for _ in range(10):
        cap.read()

    print(f"Ready. Output → {RESULTS_DIR}")
    print()

    shot = 0
    try:
        while True:
            input("Press Enter to capture  (Ctrl+C to quit) → ")
            ret, frame = cap.read()
            if not ret or frame is None:
                print("  Frame grab failed — try again.")
                continue

            shot += 1
            ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
            shot_id  = f"shot{shot:02d}_{ts}"
            print(f"  {shot_id}  ({frame.shape[1]}×{frame.shape[0]})")
            save_images(frame, shot_id, umap1, umap2, dmap1, dmap2)
            print()

    except KeyboardInterrupt:
        print("\nDone.")
    finally:
        cap.release()


if __name__ == '__main__':
    main()
