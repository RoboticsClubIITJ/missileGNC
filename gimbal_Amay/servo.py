"""
aruco_gimbal_control.py

Converts an ArUco marker's camera-frame pose (x, y, z in meters, from
aruco_gimbal_picamera2.py's marker_array) into pan (yaw) and tilt (pitch)
servo angles for a 2-axis gimbal, and drives the servos via pigpio.

Assumptions:
- Camera is mounted at (or its offset from) the gimbal's pan/tilt axis.
- Standard OpenCV camera frame: +x = right, +y = down, +z = forward.
- 2D bench setup: yaw handles left/right target offset (x), pitch
  handles up/down offset (y). z is used only as the forward distance
  in the angle formula, not tracked separately.

Geometry:
  yaw_deg   = atan2(x, z)   -> positive = target right of center
  pitch_deg = atan2(y, z)   -> positive = target below center

These are the angles the gimbal must rotate FROM its current zero point
TO point directly at the marker (i.e. this is direct/open-loop pointing,
not incremental pixel-error visual servoing).
"""

import math
import time

try:
    import pigpio
    PIGPIO_AVAILABLE = True
except ImportError:
    PIGPIO_AVAILABLE = False


# ---------------------------------------------------------------
# Gimbal geometry / calibration constants — TUNE THESE FOR YOUR RIG
# ---------------------------------------------------------------
PAN_GPIO   = 18       # BCM pin driving pan (yaw) servo
TILT_GPIO  = 19       # BCM pin driving tilt (pitch) servo

PAN_CENTER_US   = 1500   # pulse width (us) at yaw = 0 deg
TILT_CENTER_US  = 1500   # pulse width (us) at pitch = 0 deg
US_PER_DEGREE   = 10.0   # servo-specific: us change per degree of travel

PAN_LIMIT_DEG   = (-90.0, 90.0)   # mechanical yaw limits
TILT_LIMIT_DEG  = (-45.0, 45.0)   # mechanical pitch limits

# If the camera's optical axis isn't exactly aligned with the gimbal's
# mechanical zero, correct for it here (degrees).
YAW_OFFSET_DEG   = 0.0
PITCH_OFFSET_DEG = 0.0

# Exponential smoothing on commanded angle (0 = none, closer to 1 =
# heavier smoothing / slower response). Kills jitter from noisy PnP.
SMOOTHING_ALPHA = 0.3


def marker_to_angles(x, y, z):
    """Camera-frame (x, y, z) meters -> (yaw_deg, pitch_deg) to point at it."""
    if z <= 0:
        raise ValueError("z must be positive (target must be in front of camera)")

    yaw_deg = math.degrees(math.atan2(x, z)) + YAW_OFFSET_DEG
    pitch_deg = math.degrees(math.atan2(y, z)) + PITCH_OFFSET_DEG
    return yaw_deg, pitch_deg


def clamp(value, limits):
    lo, hi = limits
    return max(lo, min(hi, value))


def angle_to_pulsewidth(angle_deg, center_us):
    return center_us + angle_deg * US_PER_DEGREE


class GimbalController:
    """pigpio servo driver with clamping + smoothing."""

    def __init__(self):
        self._current_yaw = 0.0
        self._current_pitch = 0.0

        self.pi = None
        if PIGPIO_AVAILABLE:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("Could not connect to pigpiod — run: sudo pigpiod")

    def update(self, x, y, z):
        """Feed marker_array's x, y, z; drives servos; returns commanded angles."""
        target_yaw, target_pitch = marker_to_angles(x, y, z)
        target_yaw = clamp(target_yaw, PAN_LIMIT_DEG)
        target_pitch = clamp(target_pitch, TILT_LIMIT_DEG)

        self._current_yaw += SMOOTHING_ALPHA * (target_yaw - self._current_yaw)
        self._current_pitch += SMOOTHING_ALPHA * (target_pitch - self._current_pitch)

        self._drive(self._current_yaw, self._current_pitch)
        return self._current_yaw, self._current_pitch

    def _drive(self, yaw_deg, pitch_deg):
        pan_us = angle_to_pulsewidth(yaw_deg, PAN_CENTER_US)
        tilt_us = angle_to_pulsewidth(pitch_deg, TILT_CENTER_US)

        if self.pi is not None:
            self.pi.set_servo_pulsewidth(PAN_GPIO, pan_us)
            self.pi.set_servo_pulsewidth(TILT_GPIO, tilt_us)
        else:
            print(f"[SIM] pan={pan_us:.0f}us  tilt={tilt_us:.0f}us")

    def stop(self):
        if self.pi is not None:
            self.pi.set_servo_pulsewidth(PAN_GPIO, 0)
            self.pi.set_servo_pulsewidth(TILT_GPIO, 0)
            self.pi.stop()


# ---------------------------------------------------------------
# Integration point: call gimbal.update(x, y, z) inside your
# aruco_gimbal_picamera2.py loop, right where marker_array is built.
#
#   marker_array = [x, y, z, pitch, yaw, distance]
#   gimbal.update(marker_array[0], marker_array[1], marker_array[2])
# ---------------------------------------------------------------
if __name__ == "__main__":
    gimbal = GimbalController()
    test_samples = [
        (0.02, -0.01, 0.40),
        (0.05, 0.00, 0.35),
        (-0.03, 0.02, 0.30),
    ]
    try:
        for x, y, z in test_samples:
            yaw_deg, pitch_deg = gimbal.update(x, y, z)
            print(f"x={x:+.3f} y={y:+.3f} z={z:.3f}  ->  yaw={yaw_deg:+.1f}deg  pitch={pitch_deg:+.1f}deg")
            time.sleep(0.5)
    finally:
        gimbal.stop()