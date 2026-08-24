# this is the code to run and find the distance of the screen from the camera
"""
gimbal_config.py

Central config for the gimbal rig: loads the camera intrinsics produced
by calibrate_camera_picamera2.py (camera_calibration.npz) and defines
the fixed camera-to-board distance and gimbal/servo constants used by
aruco_gimbal_control.py.

Keep this file in the same folder as camera_calibration.npz, or edit
CALIBRATION_FILE below to point at it explicitly.
"""

import os
import numpy as np

# ---------------------------------------------------------------
# Camera intrinsics — loaded from calibration file
# ---------------------------------------------------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(_SCRIPT_DIR, "camera_calibration.npz")

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

# Convenience individual values pulled from the matrix, in case you need
# them elsewhere (e.g. for FOV calculations):
#     [ fx   0  cx ]
#     [  0  fy  cy ]
#     [  0   0   1 ]
FX = float(CAMERA_MATRIX[0, 0])
FY = float(CAMERA_MATRIX[1, 1])
CX = float(CAMERA_MATRIX[0, 2])
CY = float(CAMERA_MATRIX[1, 2])

print(f"Loaded camera calibration from {CALIBRATION_FILE}")
print(f"  fx={FX:.2f} fy={FY:.2f} cx={CX:.2f} cy={CY:.2f}")

# ---------------------------------------------------------------
# Fixed camera-to-board distance (meters) — MEASURE THIS ON YOUR RIG
# ---------------------------------------------------------------
# This is the constant Z used in aruco_gimbal_control.py instead of the
# (noisy) z from ArUco pose estimation, since the board sits at a known,
# fixed range from the camera.
SCREEN_DISTANCE_M = 0.40  # <-- replace with your actual measured distance

# ---------------------------------------------------------------
# ArUco marker physical size (meters) — must match aruco_gimbal_picamera2.py
# ---------------------------------------------------------------
MARKER_SIZE = 0.05

# ---------------------------------------------------------------
# Gimbal / servo hardware constants
# (kept here too so aruco_gimbal_control.py can import everything from
# one place if you prefer — currently it defines these itself; import
# from here instead if you want a single source of truth.)
# ---------------------------------------------------------------
PAN_GPIO = 18
TILT_GPIO = 19

PAN_CENTER_US = 1500
TILT_CENTER_US = 1500
US_PER_DEGREE = 10.0

PAN_LIMIT_DEG = (-90.0, 90.0)
TILT_LIMIT_DEG = (-45.0, 45.0)

YAW_OFFSET_DEG = 0.0
PITCH_OFFSET_DEG = 0.0