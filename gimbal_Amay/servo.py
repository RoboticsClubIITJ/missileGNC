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
I’ll quickly re-check the current integration points so the explanation matches the files as they exist now, including any edits made after the previous change.

Read 

Read , lines 1 to 220

Read 

I changed the folder so the intended flow is:

1. `gimbal_config.py`
   - Loads camera calibration from `camera_calibration.npz`.
   - Loads the measured screen distance from `z_distance.json`.
   - Stores shared servo settings such as GPIO pins, limits, pulse widths, and offsets.

2. `aruco_gimbal_picamera2.py`
   - Detects the ArUco marker.
   - Uses `solvePnP()` to calculate:
     - `x`, `y`, `z` position
     - marker rotation
     - straight-line distance
   - Intended to expose this through `detect_marker_pose()` so another script can call it.

3. `servo.py`
   - Imports the detector from `aruco_gimbal_picamera2.py`.
   - Captures camera frames.
   - Gets the marker coordinates.
   - Sends `x` and `y` into `GimbalController.update()`.
   - Converts the coordinates into pan and tilt angles.
   - Converts angles into servo pulse widths.
   - Uses the shared values from `gimbal_config.py` instead of duplicating servo constants.
   - Prints the calculated angles and configured screen distance.

The controller calculates angles approximately like this:

```python
yaw = atan2(x, screen_distance)
pitch = atan2(y, screen_distance)
```

It also clamps the angles to the mechanical servo limits before driving the servos.

However, I checked the current files and your later edits have reverted part of the integration. Currently, `servo.py` still imports:

```python
from aruco_gimbal_picamera2 import create_aruco_detector, detect_marker_pose
```

but the current `aruco_gimbal_picamera2.py` no longer contains those functions. Therefore, running `servo.py` will currently fail with an import error. The integration needs one more repair to restore those functions or change `servo.py` to match the current detector code.
"""

import math
import time
import socket
import struct
import cv2

try:
    from picamera2 import Picamera2
except ImportError:  # pragma: no cover - only on non-Pi hosts
    Picamera2 = None

try:
    import pigpio
    PIGPIO_AVAILABLE = True
except ImportError:
    PIGPIO_AVAILABLE = False

# Fixed camera-to-screen/board distance (meters). The z coordinate from
# ArUco pose estimation is noisy/unreliable at this range, so it is NOT
# used for the angle math — only x and y (position on the 2D board) matter.
# Replace this import with wherever you actually define/measure it.
from gimbal_config import (
    PAN_GPIO,
    TILT_GPIO,
    PAN_CENTER_US,
    TILT_CENTER_US,
    US_PER_DEGREE,
    PAN_LIMIT_DEG,
    TILT_LIMIT_DEG,
    YAW_OFFSET_DEG,
    PITCH_OFFSET_DEG,
    SCREEN_DISTANCE_M as Z_FIXED,
    SIMULINK_HOST,
    SIMULINK_PORT,
)
from aruco_gimbal_picamera2 import create_aruco_detector, detect_marker_pose


# ---------------------------------------------------------------
# Gimbal geometry / calibration constants — TUNE THESE FOR YOUR RIG
# ---------------------------------------------------------------
# Servo and geometry values are loaded from gimbal_config.py above.


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


def process_marker(marker_array, gimbal):
    """Process one detection and return servo angles plus configured distance."""
    if marker_array is None:
        return None
    delta_yaw, delta_pitch, yaw, pitch = gimbal.update(
        marker_array[0], marker_array[1]
    )
    return delta_yaw, delta_pitch, yaw, pitch, Z_FIXED


class SimulinkPoseStreamer:
    """Send [x, y, z, pitch, yaw, distance] pose packets over UDP."""

    def __init__(self, host=SIMULINK_HOST, port=SIMULINK_PORT):
        self.address = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sequence = 0

    def send(self, marker_array):
        if marker_array is None:
            return
        if len(marker_array) != 6:
            raise ValueError("marker_array must contain six pose values")
        packet = struct.pack("<6dI", *marker_array, self.sequence)
        self.socket.sendto(packet, self.address)
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF

    def close(self):
        self.socket.close()


def main():
    if Picamera2 is None:
        raise RuntimeError(
            "picamera2 is not installed on this machine. "
            "The gimbal servo script requires a Raspberry Pi camera stack."
        )

    picam2 = Picamera2()
    camera_config = picam2.create_video_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(2)

    detector, dictionary, parameters, legacy_api = create_aruco_detector()
    gimbal = GimbalController()
    simulink = SimulinkPoseStreamer()

    try:
        while True:
            frame = picam2.capture_array()
            marker_array, corners, ids = detect_marker_pose(
                frame, detector, dictionary, parameters, legacy_api
            )
            simulink.send(marker_array)
            servo_result = process_marker(marker_array, gimbal)

            if servo_result is not None:
                delta_yaw, delta_pitch, yaw, pitch, distance = servo_result
                print(
                    f"x={marker_array[0]:+.3f}m y={marker_array[1]:+.3f}m | "
                    f"delta yaw={delta_yaw:+.2f} deg, delta pitch={delta_pitch:+.2f} deg | "
                    f"yaw={yaw:+.2f} deg, pitch={pitch:+.2f} deg | "
                    f"screen distance={distance:.3f}m"
                )

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.imshow("Gimbal servo control", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        gimbal.stop()
        simulink.close()
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()