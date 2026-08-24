"""
aruco_gimbal_control.py

Converts an ArUco marker's camera-frame pose (x, y, z in meters, from
aruco_gimbal_picamera2.py's marker_array) into pan (yaw) and tilt (pitch)
servo angles for a 2-axis gimbal, and drives the servos via pigpio.

Assumptions:
- Camera is mounted at (or its offset from) the gimbal's pan/tilt axis.
- Standard OpenCV camera frame: +x = right, +y = down, +z = forward.
- Target is a flat 2D board at a KNOWN, FIXED distance from the camera
  (Z_FIXED, imported from config). The z coordinate from ArUco pose
  estimation is NOT used — it's noisy and unnecessary since the board's
  distance doesn't change.

Geometry (z = fixed distance, not measured):
  yaw_deg   = atan2(x, Z_FIXED)   -> positive = target right of center
  pitch_deg = atan2(y, Z_FIXED)   -> positive = target below center

Output is a DELTA angle: (target_angle - gimbal's current angle), i.e.
how far to move the servo from where it already is, not an absolute
angle to jump to.
"""

import math
import time

try:
    import pigpio
    PIGPIO_AVAILABLE = True
except ImportError:
    PIGPIO_AVAILABLE = False

# Fixed camera-to-screen/board distance (meters). The z coordinate from
# ArUco pose estimation is noisy/unreliable at this range, so it is NOT
# used for the angle math — only x and y (position on the 2D board) matter.
# Replace this import with wherever you actually define/measure it.
try:
    from gimbal_config import SCREEN_DISTANCE_M as Z_FIXED
except ImportError:
    Z_FIXED = 0.40  # placeholder — set to your real camera-to-screen distance


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


def marker_to_angles(x, y):
    """
    Camera-frame (x, y) meters -> (yaw_deg, pitch_deg) to point at it.

    z is NOT taken from the ArUco detection — it's fixed at Z_FIXED
    (the known camera-to-board distance), since the detected z is too
    noisy to trust here and the board is a flat 2D plane at a known range.
    """
    yaw_deg = math.degrees(math.atan2(x, Z_FIXED)) + YAW_OFFSET_DEG
    pitch_deg = math.degrees(math.atan2(y, Z_FIXED)) + PITCH_OFFSET_DEG
    return yaw_deg, pitch_deg


def clamp(value, limits):
    lo, hi = limits
    return max(lo, min(hi, value))


def angle_to_pulsewidth(angle_deg, center_us):
    return center_us + angle_deg * US_PER_DEGREE


class GimbalController:
    """
    pigpio servo driver with clamping. Tracks the gimbal's actual current
    angle and moves it by the DELTA to the new target (new_angle - prev_angle),
    rather than jumping to an absolute angle each frame.
    """

    def __init__(self, start_yaw=0.0, start_pitch=0.0):
        # Actual current physical angle of the gimbal (not a target/setpoint).
        self._current_yaw = start_yaw
        self._current_pitch = start_pitch

        self.pi = None
        if PIGPIO_AVAILABLE:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("Could not connect to pigpiod — run: sudo pigpiod")

    def update(self, x, y):
        """
        Feed marker_array's x, y (z is fixed, see Z_FIXED above).
        Computes target angle, moves the gimbal by (target - prev_angle),
        drives servos, and returns (delta_yaw, delta_pitch, new_yaw, new_pitch).
        """
        target_yaw, target_pitch = marker_to_angles(x, y)

        delta_yaw = target_yaw - self._current_yaw
        delta_pitch = target_pitch - self._current_pitch

        new_yaw = clamp(self._current_yaw + delta_yaw, PAN_LIMIT_DEG)
        new_pitch = clamp(self._current_pitch + delta_pitch, TILT_LIMIT_DEG)

        # Re-derive delta after clamping, so we never command more motion
        # than the servo can actually take.
        delta_yaw = new_yaw - self._current_yaw
        delta_pitch = new_pitch - self._current_pitch

        self._current_yaw = new_yaw
        self._current_pitch = new_pitch

        self._drive(self._current_yaw, self._current_pitch)
        return delta_yaw, delta_pitch, self._current_yaw, self._current_pitch

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
# Integration point: call gimbal.update(x, y) inside your
# aruco_gimbal_picamera2.py loop, right where marker_array is built.
# z is ignored (fixed distance, see Z_FIXED above).
#
#   marker_array = [x, y, z, pitch, yaw, distance]
#   gimbal.update(marker_array[0], marker_array[1])
# ---------------------------------------------------------------
if __name__ == "__main__":
    gimbal = GimbalController()
    test_samples = [
        (0.02, -0.01),
        (0.05, 0.00),
        (-0.03, 0.02),
    ]
    try:
        for x, y in test_samples:
            d_yaw, d_pitch, yaw_deg, pitch_deg = gimbal.update(x, y)
            print(f"x={x:+.3f} y={y:+.3f}  ->  delta_yaw={d_yaw:+.2f}deg  delta_pitch={d_pitch:+.2f}deg  "
                  f"(abs yaw={yaw_deg:+.1f}deg pitch={pitch_deg:+.1f}deg)")
            time.sleep(0.5)
    finally:
        gimbal.stop()