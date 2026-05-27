#!/usr/bin/env python3
"""
Press Enter → saves 5 images to results/images/.
  1_raw.png              — straight from camera
  2_undistorted.png      — fisheye corrected
  3_exaggerated.png      — barrel distortion applied to undistorted (full frame, no black)
  4_gray_820x616.png     — undistorted → grayscale → 820×616
  5_apriltag.png         — image 4 + thick XYZ axes + semi-transparent approach arrow
Ctrl+C to quit.  Stop mission.launch.py first.
"""

import math, os, sys, cv2, numpy as np
from datetime import datetime

RESULTS_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'results', 'images')
)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
DISTORT_SCALE = 1.5    # image 3 — barrel strength, try 1.2 – 3.0
APPROACH_DIST = 0.9   # image 5 — standoff from box centre (metres)
ARROW_ALPHA   = 0.90   # image 5 — arrow opacity: 0.0 invisible → 1.0 solid
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# Camera matrix at full resolution (projection_matrix, scale_factor=1)
K = np.array([
    [1813.000558,    0.,         1634.163912],
    [   0.,       1826.621610,  1258.598589],
    [   0.,          0.,            1.     ],
], dtype=np.float64)
D   = np.array([-0.022401, -0.022708, 0.014203, -0.009313], dtype=np.float64)
DIM = (3280, 2464)

# Undistort maps — built once
_umap1, _umap2 = cv2.fisheye.initUndistortRectifyMap(
    K, D, np.eye(3), K, DIM, cv2.CV_32FC1
)

# AprilTag detector (same family as mission stack)
_DICT    = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
DETECTOR = cv2.aruco.ArucoDetector(_DICT, cv2.aruco.DetectorParameters())
TAG_SIZE = 0.103   # metres

# K scaled for 820×616
_s = 820 / DIM[0]
K_SMALL = np.array([
    [K[0,0]*_s, 0.,         K[0,2]*_s],
    [0.,        K[1,1]*_s,  K[1,2]*_s],
    [0.,        0.,         1.        ],
], dtype=np.float64)

# Box geometry — mirrors box_estimator.py
BOX_LENGTH, BOX_WIDTH = 0.25, 0.20

def _Tz(d):
    M=np.eye(4); M[2,3]=d; return M
def _Rx(deg):
    r=math.radians(deg); c,s=math.cos(r),math.sin(r)
    M=np.eye(4); M[1,1]=c; M[1,2]=-s; M[2,1]=s; M[2,2]=c; return M
def _Ry(deg):
    r=math.radians(deg); c,s=math.cos(r),math.sin(r)
    M=np.eye(4); M[0,0]=c; M[0,2]=s; M[2,0]=-s; M[2,2]=c; return M

_M2B = {
    0: _Tz( BOX_LENGTH/2)         @ _Rx(90),
    1: _Tz( BOX_LENGTH/2)         @ _Ry(180) @ _Rx(90),
    2: _Tz( BOX_WIDTH/2 + 0.003)  @ _Ry(90)  @ _Rx(90),
    3: _Tz(-BOX_WIDTH/2)          @ _Ry(-90) @ _Rx(90),
}


# ── GStreamer pipeline (from rectified_parking_default.py) ──────────────
def _gst():
    w, h = DIM
    return (
        f"nvarguscamerasrc sensor-id=0 ! "
        f"video/x-raw(memory:NVMM), width=(int){w}, height=(int){h}, framerate=(fraction)21/1 ! "
        f"nvvidconv flip-method=0 ! "
        f"video/x-raw, width=(int){w}, height=(int){h}, format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! appsink drop=1"
    )


# ── Image 3: exaggerated barrel distortion ──────────────────────────────
def _exaggerate(undistorted):
    """
    Apply forward fisheye distortion (D * DISTORT_SCALE) to the undistorted image.
    Corners are filled with BORDER_REFLECT so the full frame stays rectangular
    and no black appears regardless of scale.
    """
    h, w = undistorted.shape[:2]
    xx, yy = np.meshgrid(np.arange(w, dtype=np.float32),
                         np.arange(h, dtype=np.float32))
    pts = np.stack([xx.ravel(), yy.ravel()], axis=1).reshape(-1, 1, 2)
    src = cv2.fisheye.undistortPoints(
        pts, K, D * DISTORT_SCALE, P=K
    ).reshape(h, w, 2)
    return cv2.remap(undistorted, src[..., 0], src[..., 1],
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)


# ── Image 5: AprilTag axes + approach arrow ─────────────────────────────
def _draw_axes(img, corners, tag_id):
    half = TAG_SIZE / 2
    obj  = np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]], np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, corners.reshape(4,2).astype(np.float64),
                                   K_SMALL, np.zeros(4))
    if not ok:
        return None, None

    R, _ = cv2.Rodrigues(rvec)
    # Negate Z column only: X=right, Y=down, Z=into-tag (right-hand rule in camera frame).
    # Cannot convert back via Rodrigues (det=-1), so draw axes manually.
    R[:, 2] = -R[:, 2]

    ctr = tvec.ravel()
    sc  = TAG_SIZE * 1.3

    def _proj(pts):
        p, _ = cv2.projectPoints(
            np.array(pts, dtype=np.float64).reshape(-1, 1, 3),
            np.zeros(3), np.zeros(3), K_SMALL, np.zeros(4))
        return p.reshape(-1, 2).astype(int)

    o2d     = tuple(_proj([ctr])[0])
    tips_2d = _proj([ctr + R[:, i] * sc for i in range(3)])

    font = cv2.FONT_HERSHEY_SIMPLEX
    for i, (color, lbl) in enumerate([
            ((0, 0, 255), 'X'), ((0, 255, 0), 'Y'), ((255, 0, 0), 'Z')]):
        tip = tuple(tips_2d[i])
        cv2.arrowedLine(img, o2d, tip, color, 5, tipLength=0.25)
        cv2.putText(img, lbl, (tip[0] + 4, tip[1]), font, 0.6, (0, 0, 0), 4)
        cv2.putText(img, lbl, (tip[0] + 4, tip[1]), font, 0.6, color, 2)

    return R, tvec   # return corrected R matrix (not rvec)


def _draw_approach(overlay, R, tvec, tag_id):
    if tag_id not in _M2B:
        return
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = tvec.ravel()
    box = T @ _M2B[tag_id]
    bp  = box[:3, 3]                          # box centre in camera frame
    ap  = bp - APPROACH_DIST * box[:3, 0]     # standoff along -X axis

    if bp[2] <= 0 or ap[2] <= 0:
        return

    pts, _ = cv2.projectPoints(
        np.array([[bp],[ap]], np.float64),
        np.zeros(3), np.zeros(3), K_SMALL, np.zeros(4))
    bp2, ap2 = pts.reshape(-1, 2).astype(int)

    cv2.arrowedLine(overlay, tuple(bp2), tuple(ap2), (0,0,255), 6, tipLength=0.15)
    cv2.circle(overlay, tuple(bp2), 10, (0,120,255), -1)
    cv2.circle(overlay, tuple(ap2), 10, (0,  0,255), -1)

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(overlay, f'{APPROACH_DIST:.2f}m', (ap2[0]+10, ap2[1]), font, 0.65, (0,0,0),   4)
    cv2.putText(overlay, f'{APPROACH_DIST:.2f}m', (ap2[0]+10, ap2[1]), font, 0.65, (0,0,255), 2)


# ── Main save routine ───────────────────────────────────────────────────
def save(raw, shot_id):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    p = os.path.join(RESULTS_DIR, shot_id)

    # 1 — raw
    cv2.imwrite(f'{p}_1_raw.png', raw)
    print('  [1] raw')

    # 2 — undistorted
    undist = cv2.remap(raw, _umap1, _umap2, cv2.INTER_LINEAR)
    cv2.imwrite(f'{p}_2_undistorted.png', undist)
    print('  [2] undistorted')

    # 3 — exaggerated barrel distortion (full frame, DISTORT_SCALE applied)
    exag = _exaggerate(undist)
    cv2.imwrite(f'{p}_3_exaggerated_x{DISTORT_SCALE}.png', exag)
    print(f'  [3] exaggerated ×{DISTORT_SCALE}')

    # 4 — grayscale 820×616 from undistorted
    gray = cv2.cvtColor(
        cv2.resize(undist, (820, 616), interpolation=cv2.INTER_AREA),
        cv2.COLOR_BGR2GRAY)
    cv2.imwrite(f'{p}_4_gray_820x616.png', gray)
    print('  [4] gray 820×616')

    # 5 — AprilTag axes + approach arrow on image 4
    img5 = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    corners, ids, _ = DETECTOR.detectMarkers(gray)
    solves = []
    if ids is not None:
        for c, tid in zip(corners, ids.flatten()):
            if int(tid) == 3:
                continue  # marker 3 excluded from drawing
            R_mat, tv = _draw_axes(img5, c[0], int(tid))
            if R_mat is not None:
                solves.append((int(tid), R_mat, tv))
        print(f'  [5] detected tags: {ids.flatten().tolist()}')
    else:
        cv2.putText(img5, 'No AprilTag', (10,40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 3)
        print('  [5] no tags detected')

    if solves:
        ov = img5.copy()
        for tid, R_mat, tv in solves:
            _draw_approach(ov, R_mat, tv, tid)
        cv2.addWeighted(ov, ARROW_ALPHA, img5, 1 - ARROW_ALPHA, 0, img5)

    cv2.imwrite(f'{p}_5_apriltag.png', img5)
    print('  [5] apriltag axes + approach arrow\n')


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    print(f'Output → {RESULTS_DIR}')
    print(f'Distort scale: {DISTORT_SCALE}  |  Approach: {APPROACH_DIST}m  |  Alpha: {ARROW_ALPHA}')
    print('Stop mission.launch.py first.\n')

    cap = cv2.VideoCapture(_gst(), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print('ERROR: camera failed to open'); sys.exit(1)

    for _ in range(30):
        cap.read()
    print('Ready.\n')

    shot = 0
    try:
        while True:
            input('Press Enter to capture (Ctrl+C to quit) → ')
            ret, frame = cap.read()
            if not ret:
                print('  grab failed, try again'); continue
            shot += 1
            ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
            sid = f'shot{shot:02d}_{ts}'
            print(f'  {sid}  {frame.shape[1]}×{frame.shape[0]}')
            save(frame, sid)
    except KeyboardInterrupt:
        print('\nDone.')
    finally:
        cap.release()


if __name__ == '__main__':
    main()
