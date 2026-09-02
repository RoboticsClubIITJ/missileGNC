

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
    SIMULINK_HOST,
    SIMULINK_PORT,
)
from aruco_gimbal_picamera2 import create_aruco_detector, detect_marker_pose


# ---------------------------------------------------------------
# Gimbal geometry / calibration constants — TUNE THESE FOR YOUR RIG
# ---------------------------------------------------------------
# Servo and geometry values are loaded from gimbal_config.py above.


def offset_to_angles(x, y, z):
    """
    Convert the current camera-frame marker vector into pan/tilt correction
    angles. x, y, and z are the per-frame ArUco translation in meters. Pan is
    measured in the horizontal x-z plane; tilt is measured from that plane.

    The horizontal camera x-axis is inverted relative to the mechanical pan
    servo direction on many gimbal builds, so the sign is flipped here to make
    left/right alignment match the physical rig.
    """
    if z <= 0:
        raise ValueError(f"Marker must be in front of the camera; received z={z:.4f}m")

    horizontal_range = math.hypot(x, z)
    yaw_deg = -math.degrees(math.atan2(x, z)) + YAW_OFFSET_DEG
    pitch_deg = math.degrees(math.atan2(y, horizontal_range)) + PITCH_OFFSET_DEG
    return yaw_deg, pitch_deg


def clamp(value, limits):
    lo, hi = limits
    return max(lo, min(hi, value))


def angle_to_pulsewidth(angle_deg, center_us):
    return center_us + angle_deg * US_PER_DEGREE


class GimbalController:
    """Drive the gimbal from the marker's current 3D camera-frame vector."""

    def __init__(self, start_yaw=0.0, start_pitch=0.0):
        # Actual current physical angle of the gimbal (not a target/setpoint).
        self._current_yaw = start_yaw
        self._current_pitch = start_pitch

        self.pi = None
        if PIGPIO_AVAILABLE:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("Could not connect to pigpiod — run: sudo pigpiod")

        # Start both servos at their center/neutral position so the rig begins
        # in the mean mechanical pose instead of an arbitrary default.
        self._drive(self._current_yaw, self._current_pitch)

    def update(self, x, y, z):
        """
        Feed the current frame's marker vector in meters. The vector is
        converted to an angular error using its live z value, then applied
        incrementally to the current servo angles.
        """
        correction_yaw, correction_pitch = offset_to_angles(x, y, z)

        new_yaw = clamp(self._current_yaw + correction_yaw, PAN_LIMIT_DEG)
        new_pitch = clamp(self._current_pitch + correction_pitch, TILT_LIMIT_DEG)

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
    """Process one detection and return servo angles plus measured distance."""
    if marker_array is None:
        return None
    delta_yaw, delta_pitch, yaw, pitch = gimbal.update(
        marker_array[0], marker_array[1], marker_array[2]
    )
    return delta_yaw, delta_pitch, yaw, pitch, marker_array[5]


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
                    f"marker distance={distance:.3f}m"
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