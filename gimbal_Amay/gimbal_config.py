"""
gimbal_config.py

Central config for the gimbal rig: loads the camera intrinsics produced
by calibrate_camera_picamera2.py (camera_calibration.npz). The runtime
controller uses the per-frame ArUco z coordinate; the optional distance
calibration below is retained for diagnostics and compatibility.

Z-distance calibration workflow:
    1. Place the ArUco marker at the center of the board, directly in
       front of the camera (this becomes your gimbal's "zero" point).
    2. Run:  python gimbal_config.py --calibrate
       This detects the marker for a few seconds, averages its z, and
       saves it to z_distance.json.
        3. The cached value is available for diagnostics; servo.py does not
             use it for runtime angle calculations.

Keep this file in the same folder as camera_calibration.npz.
"""

import os
import json
import time
import math
import numpy as np

# Load environment variables from .env if available.
# If python-dotenv is not installed, continue with safe defaults.
try:
    from dotenv import load_dotenv
except ImportError:  # pragma: no cover - optional dependency
    def load_dotenv(*args, **kwargs):
        return False

load_dotenv()

SIMULINK_HOST = os.getenv("SIMULINK_HOST", "127.0.0.1")
SIMULINK_PORT = int(os.getenv("SIMULINK_PORT", "25000"))

# ---------------------------------------------------------------
# Camera intrinsics — loaded from calibration file
# ---------------------------------------------------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(_SCRIPT_DIR, "camera_calibration.npz")
Z_CACHE_FILE = os.path.join(_SCRIPT_DIR, "z_distance.json")

if not os.path.exists(CALIBRATION_FILE):
    raise FileNotFoundError(
        f"Camera calibration file not found at {CALIBRATION_FILE}. "
        f"Run calibrate_camera_picamera2.py first."
    )

with np.load(CALIBRATION_FILE) as _calib:
    # Accept either key naming convention, matching aruco_gimbal_picamera2.py
    _matrix_key = "camera_matrix" if "camera_matrix" in _calib else "mtx"
    _dist_key = "dist_coeffs" if "dist_coeffs" in _calib else "dist"

    CAMERA_MATRIX = _calib[_matrix_key].astype(np.float32)
    DIST_COEFFS = _calib[_dist_key].reshape(-1, 1).astype(np.float32)

if CAMERA_MATRIX.shape != (3, 3) or not np.isfinite(CAMERA_MATRIX).all():
    raise ValueError(f"Invalid camera_matrix in {CALIBRATION_FILE}")
if DIST_COEFFS.shape[0] < 4 or not np.isfinite(DIST_COEFFS).all():
    raise ValueError(f"Invalid dist_coeffs in {CALIBRATION_FILE}")

FX = float(CAMERA_MATRIX[0, 0])
FY = float(CAMERA_MATRIX[1, 1])
CX = float(CAMERA_MATRIX[0, 2])
CY = float(CAMERA_MATRIX[1, 2])

print(f"Loaded camera calibration from {CALIBRATION_FILE}")
print(f"  fx={FX:.2f} fy={FY:.2f} cx={CX:.2f} cy={CY:.2f}")

# ---------------------------------------------------------------
# ArUco marker physical size (meters) — must match aruco_gimbal_picamera2.py
# ---------------------------------------------------------------
MARKER_SIZE = 0.0738


def _detect_marker_pose(gray, dictionary, parameters, legacy_api, marker_points):
    """One-shot detection + PnP on a grayscale frame. Returns (x, y, z) or None."""
    import cv2

    if not legacy_api:
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    if ids is None or len(ids) == 0:
        return None

    success, rvec, tvec = cv2.solvePnP(
        marker_points, corners[0][0], CAMERA_MATRIX, DIST_COEFFS,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not success:
        return None

    t = tvec.flatten()
    return float(t[0]), float(t[1]), float(t[2])


def calibrate_z_distance(num_samples=30, max_center_offset_m=0.03):
    """
    Opens the Pi camera, detects the ArUco marker for up to ~num_samples
    frames, and averages z (distance) into SCREEN_DISTANCE_M. The marker
    should be placed dead-center in the camera's view — warns if the
    average x/y offset suggests it isn't.

    Saves the result to Z_CACHE_FILE and returns the measured distance.
    """
    import cv2
    from picamera2 import Picamera2

    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()
    time.sleep(2)  # let AE/AWB settle

    if hasattr(cv2.aruco, "ArucoDetector"):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        legacy_api = False
    else:
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        legacy_api = True

    marker_points = np.array([
        [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
        [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
        [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
        [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
    ], dtype=np.float32)

    samples = []
    try:
        print(f"Calibrating Z: place marker at board center, hold still ({num_samples} samples)...")
        while len(samples) < num_samples:
            frame = picam2.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            pose = _detect_marker_pose(gray, dictionary, parameters, legacy_api, marker_points)
            if pose is not None:
                samples.append(pose)
            time.sleep(0.05)
    finally:
        picam2.stop()

    if not samples:
        raise RuntimeError("No marker detected during Z calibration — check placement/lighting.")

    xs, ys, zs = zip(*samples)
    mean_x, mean_y, mean_z = sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs)

    if math.hypot(mean_x, mean_y) > max_center_offset_m:
        print(
            f"WARNING: marker average offset (x={mean_x:+.3f}, y={mean_y:+.3f}) is not "
            f"close to center. Re-center the marker and recalibrate for an accurate zero."
        )

    print(f"Measured Z distance: {mean_z:.4f} m (from {len(samples)} samples)")

    with open(Z_CACHE_FILE, "w") as f:
        json.dump({"screen_distance_m": mean_z, "samples": len(samples)}, f, indent=2)

    return mean_z


def _load_cached_z():
    if os.path.exists(Z_CACHE_FILE):
        with open(Z_CACHE_FILE) as f:
            return json.load(f)["screen_distance_m"]
    return None


# ---------------------------------------------------------------
# Optional representative distance retained for diagnostics and compatibility.
# servo.py uses the per-frame ArUco z value instead.
# ---------------------------------------------------------------
_cached_z = _load_cached_z()
if _cached_z is not None:
    SCREEN_DISTANCE_M = _cached_z
    print(f"Loaded cached Z distance: {SCREEN_DISTANCE_M:.4f} m (from {Z_CACHE_FILE})")
else:
    SCREEN_DISTANCE_M = 1
    # placeholder until calibrated
    print(
        f"No cached Z distance found. Using placeholder {SCREEN_DISTANCE_M} m — "
        f"run `python gimbal_config.py --calibrate` to measure the real value."
    )

# ---------------------------------------------------------------
# Gimbal / servo hardware constants
# ---------------------------------------------------------------
PAN_GPIO = 18
TILT_GPIO = 19

# Bench-calibrated starting values from servo sweep:
# 1100 -> 1300 us gave approx 25 deg travel, so ~8 us/deg.
PAN_CENTER_US = 1500
TILT_CENTER_US = 1500
US_PER_DEGREE = 8.0
SERVO_MIN_PULSE_US = 1000
SERVO_MAX_PULSE_US = 2000
STARTUP_HOLD_S = 5.0

# Feedback tuning for fast tracking without uncontrolled jumps. Corrections
# are filtered, reduced by CONTROL_GAIN, ignored inside the deadband, and
# limited per camera frame.
# PID gains operate on the filtered angular error in degrees.
CONTROL_GAIN = 0.45
INTEGRAL_GAIN = 0.0
DERIVATIVE_GAIN = 0.0
DERIVATIVE_LIMIT_DEG_S = 45.0
INTEGRAL_LIMIT_DEG_S = 2.0
ERROR_DEADBAND_DEG = 0.80
MIN_COMMAND_DEG = 0.05
MAX_STEP_DEG = 4.0
CONTROL_UPDATE_PERIOD_S = 0.02
ANGLE_FILTER_ALPHA = 0.35

# Detection performance controls. Lower DETECTION_SCALE is faster but makes
# very small markers harder to detect; 0.60 is a fast balanced setting.
DETECTION_SCALE = 0.60
CORNER_REFINE_ITERATIONS = 10
CORNER_REFINE_EPSILON = 0.01

# Marker 0 is a one-time, size-independent origin reference. The search
# controller uses image-space error until this marker is centered.
ORIGIN_MARKER_ID = 0
ORIGIN_CENTER_TOLERANCE_PX = 18
SEARCH_STEP_DEG = 1.5
SEARCH_TILT_STEP_DEG = 2.0
SEARCH_UPDATE_PERIOD_S = 0.10
ORIGIN_SEARCH_LIMIT_DEG = 60.0
ORIGIN_SEARCH_TILT_LIMIT_DEG = 45.0

# Keep commanded motion inside the tested mechanical search envelope.
PAN_LIMIT_DEG = (-60.0, 60.0)
TILT_LIMIT_DEG = (-45.0, 45.0)

# Keep zero offsets at neutral until the physical axis direction is checked.
YAW_OFFSET_DEG = 0.0
PITCH_OFFSET_DEG = 0.0

# Simulink UDP receiver settings. Set SIMULINK_HOST to the IP address of the
# computer running anjaneya_V7.slx.
# Defaults are safe for local testing when no .env file is present.


if __name__ == "__main__":
    import sys
    if "--calibrate" in sys.argv:
        calibrate_z_distance()
    else:
        print("Run with --calibrate to measure and cache the Z distance.")