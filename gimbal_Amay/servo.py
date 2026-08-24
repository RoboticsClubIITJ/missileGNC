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
  Each frame's (x, y) from ArUco is the marker's offset in the CURRENT
  camera frame — i.e. relative to wherever the gimbal is pointed right
  now, not relative to the original zero. Since the camera moves with
  the gimbal, these offsets must be ACCUMULATED across frames to get
  the marker's true offset from the original boresight:

      x_abs += x_new      (running total, meters)
      y_abs += y_new

  Then:
      yaw_deg   = atan2(x_abs, Z_FIXED)   -> absolute angle from start
      pitch_deg = atan2(y_abs, Z_FIXED)

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


def offset_to_angles(x_abs, y_abs):
    """
    Accumulated camera-frame offset (x_abs, y_abs) meters, measured from
    the ORIGINAL boresight -> (yaw_deg, pitch_deg) absolute angle to
    point at it. z is fixed at Z_FIXED (known board distance).
    """
    yaw_deg = math.degrees(math.atan2(x_abs, Z_FIXED)) + YAW_OFFSET_DEG
    pitch_deg = math.degrees(math.atan2(y_abs, Z_FIXED)) + PITCH_OFFSET_DEG
    return yaw_deg, pitch_deg


def clamp(value, limits):
    lo, hi = limits
    return max(lo, min(hi, value))


def angle_to_pulsewidth(angle_deg, center_us):
    return center_us + angle_deg * US_PER_DEGREE


class GimbalController:
    """
    pigpio servo driver with clamping. Accumulates each frame's camera-
    relative (x, y) offset into an absolute offset from the original
    boresight (since the camera moves with the gimbal), converts that to
    an absolute target angle, and moves the gimbal by the DELTA from its
    previous angle (target - prev_angle) rather than jumping to an
    absolute angle each frame.
    """

    def __init__(self, start_yaw=0.0, start_pitch=0.0):
        # Running total offset of the marker from the ORIGINAL boresight,
        # in meters at the Z_FIXED plane. t=0: marker is at (x0, y0) -> this
        # starts at 0 and accumulates as the gimbal (and camera) moves.
        self._accum_x = 0.0
        self._accum_y = 0.0

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
        Feed marker_array's x, y for the CURRENT frame (offset relative to
        wherever the camera is pointed right now — z is fixed, see Z_FIXED).
        Accumulates it into the absolute offset, computes the absolute
        target angle, moves the gimbal by (target - prev_angle), drives
        servos, and returns (delta_yaw, delta_pitch, new_yaw, new_pitch).
        """
        self._accum_x += x
        self._accum_y += y

        target_yaw, target_pitch = offset_to_angles(self._accum_x, self._accum_y)

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
    # Each tuple is x1, y1 as measured in the CURRENT camera frame for that
    # frame — i.e. what marker_array[0], marker_array[1] gives you each loop,
    # already relative to wherever the gimbal is currently pointed.
    test_samples = [
        (0.02, -0.01),   # t=0: marker at x0,y0 -> gimbal turns to center it
        (0.01, 0.00),    # t=1: marker moved slightly, re-detected offset
        (-0.03, 0.02),   # t=2: marker moved further
    ]
    try:
        for x, y in test_samples:
            d_yaw, d_pitch, yaw_deg, pitch_deg = gimbal.update(x, y)
            print(f"frame x={x:+.3f} y={y:+.3f}  ->  delta_yaw={d_yaw:+.2f}deg  delta_pitch={d_pitch:+.2f}deg  "
                  f"(abs yaw={yaw_deg:+.1f}deg pitch={pitch_deg:+.1f}deg)")
            time.sleep(0.5)
    finally:
        gimbal.stop()