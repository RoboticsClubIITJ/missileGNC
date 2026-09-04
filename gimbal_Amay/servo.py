

import math
import time
import socket
import struct
import csv
import os
import cv2
import numpy as np

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
    CONTROL_GAIN,
    INTEGRAL_GAIN,
    DERIVATIVE_GAIN,
    DERIVATIVE_LIMIT_DEG_S,
    INTEGRAL_LIMIT_DEG_S,
    ERROR_DEADBAND_DEG,
    MIN_COMMAND_DEG,
    MAX_STEP_DEG,
    CONTROL_UPDATE_PERIOD_S,
    ANGLE_FILTER_ALPHA,
    FX,
    FY,
    CX,
    CY,
    ORIGIN_MARKER_ID,
    ORIGIN_CENTER_TOLERANCE_PX,
    SEARCH_STEP_DEG,
    SEARCH_TILT_STEP_DEG,
    SEARCH_UPDATE_PERIOD_S,
    ORIGIN_SEARCH_LIMIT_DEG,
    ORIGIN_SEARCH_TILT_LIMIT_DEG,
    SIMULINK_HOST,
    SIMULINK_PORT,
)
from aruco_gimbal_picamera2 import (
    create_aruco_detector,
    detect_marker_center,
    detect_marker_pose,
)


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
        self._filtered_yaw_error = 0.0
        self._filtered_pitch_error = 0.0
        self._yaw_integral = 0.0
        self._pitch_integral = 0.0
        self._previous_yaw_error = 0.0
        self._previous_pitch_error = 0.0
        self._previous_update_time = time.perf_counter()
        self._last_search_time = self._previous_update_time

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
        yaw_error, pitch_error = offset_to_angles(x, y, z)
        return self.update_direction(yaw_error, pitch_error)

    def update_direction(self, yaw_error, pitch_error):
        """Apply a pan/tilt correction expressed as angular error."""
        self._filtered_yaw_error = (
            ANGLE_FILTER_ALPHA * yaw_error
            + (1.0 - ANGLE_FILTER_ALPHA) * self._filtered_yaw_error
        )
        self._filtered_pitch_error = (
            ANGLE_FILTER_ALPHA * pitch_error
            + (1.0 - ANGLE_FILTER_ALPHA) * self._filtered_pitch_error
        )

        now = time.perf_counter()
        elapsed = now - self._previous_update_time
        if elapsed < CONTROL_UPDATE_PERIOD_S:
            return 0.0, 0.0, self._current_yaw, self._current_pitch
        dt = max(1.0 / 120.0, min(now - self._previous_update_time, 0.2))
        self._previous_update_time = now

        correction_yaw = self._pid_correction(
            self._filtered_yaw_error,
            self._previous_yaw_error,
            self._yaw_integral,
            dt,
        )
        correction_pitch = self._pid_correction(
            self._filtered_pitch_error,
            self._previous_pitch_error,
            self._pitch_integral,
            dt,
        )
        self._yaw_integral = correction_yaw[1]
        self._pitch_integral = correction_pitch[1]
        self._previous_yaw_error = self._filtered_yaw_error
        self._previous_pitch_error = self._filtered_pitch_error
        correction_yaw = correction_yaw[0]
        correction_pitch = correction_pitch[0]

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

    def reset_control_state(self):
        """Clear history so a new tracking state starts without a PID kick."""
        self._filtered_yaw_error = 0.0
        self._filtered_pitch_error = 0.0
        self._yaw_integral = 0.0
        self._pitch_integral = 0.0
        self._previous_yaw_error = 0.0
        self._previous_pitch_error = 0.0
        self._previous_update_time = time.perf_counter()

    def search_step(self, yaw_direction, pitch_direction):
        """Move slowly in a horizontal raster while searching for marker 0."""
        now = time.perf_counter()
        if now - self._last_search_time < SEARCH_UPDATE_PERIOD_S:
            return self._current_yaw, self._current_pitch, False, False
        self._last_search_time = now

        next_yaw = self._current_yaw + yaw_direction * SEARCH_STEP_DEG
        hit_yaw_limit = not (-ORIGIN_SEARCH_LIMIT_DEG <= next_yaw <= ORIGIN_SEARCH_LIMIT_DEG)
        self._current_yaw = clamp(
            next_yaw, (-ORIGIN_SEARCH_LIMIT_DEG, ORIGIN_SEARCH_LIMIT_DEG)
        )

        hit_pitch_limit = False
        if hit_yaw_limit:
            next_pitch = self._current_pitch + pitch_direction * SEARCH_TILT_STEP_DEG
            hit_pitch_limit = not (
                -ORIGIN_SEARCH_TILT_LIMIT_DEG
                <= next_pitch
                <= ORIGIN_SEARCH_TILT_LIMIT_DEG
            )
            self._current_pitch = clamp(
                next_pitch,
                (-ORIGIN_SEARCH_TILT_LIMIT_DEG, ORIGIN_SEARCH_TILT_LIMIT_DEG),
            )
        self._drive(self._current_yaw, self._current_pitch)
        return self._current_yaw, self._current_pitch, hit_yaw_limit, hit_pitch_limit

    @property
    def current_angles(self):
        return self._current_yaw, self._current_pitch

    @staticmethod
    def _pid_correction(error_deg, previous_error_deg, integral, dt):
        if abs(error_deg) <= ERROR_DEADBAND_DEG:
            return 0.0, 0.0

        integral = clamp(
            integral + error_deg * dt,
            (-INTEGRAL_LIMIT_DEG_S, INTEGRAL_LIMIT_DEG_S),
        )
        derivative = clamp(
            (error_deg - previous_error_deg) / dt,
            (-DERIVATIVE_LIMIT_DEG_S, DERIVATIVE_LIMIT_DEG_S),
        )
        correction_before_limit = (
            CONTROL_GAIN * error_deg
            + INTEGRAL_GAIN * integral
            + DERIVATIVE_GAIN * derivative
        )
        correction = clamp(
            correction_before_limit, (-MAX_STEP_DEG, MAX_STEP_DEG)
        )
        if abs(correction) < MIN_COMMAND_DEG:
            correction = 0.0
        if correction != correction_before_limit:
            integral -= error_deg * dt
        return correction, integral

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


def process_marker(marker_array, gimbal, origin_angles):
    """Process one detection and return servo angles plus measured distance."""
    if marker_array is None:
        return None
    origin_yaw, origin_pitch = origin_angles
    delta_yaw, delta_pitch, yaw, pitch = gimbal.update(
        marker_array[0], marker_array[1], marker_array[2]
    )
    relative_yaw = yaw - origin_yaw
    relative_pitch = pitch - origin_pitch
    return delta_yaw, delta_pitch, relative_yaw, relative_pitch, marker_array[5]


def pose_relative_to_origin(marker_array, origin_angles, current_angles, origin_pose):
    """Express a target pose as a metric vector in the latched origin frame."""
    if marker_array is None:
        return None

    origin_yaw, origin_pitch = origin_angles
    current_yaw, current_pitch = current_angles
    target_yaw_error, target_pitch_error = offset_to_angles(
        marker_array[0], marker_array[1], marker_array[2]
    )
    relative_yaw = math.radians(current_yaw + target_yaw_error - origin_yaw)
    relative_pitch = math.radians(current_pitch + target_pitch_error - origin_pitch)
    distance = marker_array[5]
    horizontal_distance = distance * math.cos(relative_pitch)
    x = -horizontal_distance * math.sin(relative_yaw)
    y = -distance * math.sin(relative_pitch)
    z = horizontal_distance * math.cos(relative_yaw)
    return [
        float(x - origin_pose[0]),
        float(y + origin_pose[1]),
        float(z - origin_pose[2]),
        marker_array[3],
        math.degrees(relative_yaw),
        float(distance),
    ]


def pose_relative_to_marker(target_pose, origin_pose, origin_rotation):
    """Express a target center in the origin marker's coordinate frame."""
    camera_delta = np.asarray(target_pose[:3]) - np.asarray(origin_pose[:3])
    marker_delta = origin_rotation.T @ camera_delta
    x, camera_y, marker_z = marker_delta.tolist()
    y = camera_y
    z = -marker_z
    distance = target_pose[5]
    horizontal_distance = math.hypot(x, z)
    return [
        float(x),
        float(y),
        float(z),
        target_pose[3],
        math.degrees(math.atan2(-x, z)),
        float(distance),
    ]


def draw_target_hud(frame, marker_array, servo_result, response_ms, fps, status):
    """Draw a readable targeting HUD over the live camera frame."""
    height, width = frame.shape[:2]
    center_x, center_y = width // 2, height // 2
    accent = (70, 220, 90)
    warning = (40, 180, 255)
    white = (235, 245, 245)
    panel = (18, 27, 29)

    # Center reticle shows the camera boresight even when no marker is visible.
    cv2.line(frame, (center_x - 28, center_y), (center_x - 8, center_y), accent, 1)
    cv2.line(frame, (center_x + 8, center_y), (center_x + 28, center_y), accent, 1)
    cv2.line(frame, (center_x, center_y - 28), (center_x, center_y - 8), accent, 1)
    cv2.line(frame, (center_x, center_y + 8), (center_x, center_y + 28), accent, 1)
    cv2.circle(frame, (center_x, center_y), 4, accent, 1)

    if marker_array is not None:
        # The marker center is projected from its pose, so the lock indicator
        # remains useful even when the marker is not exactly at image center.
        marker_x = int(round(CX + FX * marker_array[0] / marker_array[2]))
        marker_y = int(round(CY + FY * marker_array[1] / marker_array[2]))
        marker_x = max(0, min(width - 1, marker_x))
        marker_y = max(0, min(height - 1, marker_y))
        bracket = 22
        cv2.line(frame, (marker_x - bracket, marker_y - bracket),
                 (marker_x - 8, marker_y - bracket), accent, 2)
        cv2.line(frame, (marker_x + 8, marker_y - bracket),
                 (marker_x + bracket, marker_y - bracket), accent, 2)
        cv2.line(frame, (marker_x - bracket, marker_y + bracket),
                 (marker_x - bracket, marker_y + 8), accent, 2)
        cv2.line(frame, (marker_x + bracket, marker_y + bracket),
                 (marker_x + bracket, marker_y + 8), accent, 2)
        cv2.putText(frame, "TARGET LOCKED", (marker_x + 28, marker_y - 28),
                    cv2.FONT_HERSHEY_DUPLEX, 0.55, accent, 1, cv2.LINE_AA)
        cv2.line(frame, (center_x, center_y), (marker_x, marker_y), accent, 1)
    else:
        cv2.putText(frame, status, (20, 38),
                    cv2.FONT_HERSHEY_DUPLEX, 0.62, warning, 1, cv2.LINE_AA)

    # Compact telemetry panel keeps the image visible while making timing
    # and targeting state immediately apparent during hardware testing.
    overlay = frame.copy()
    cv2.rectangle(overlay, (14, 14), (260, 126), panel, -1)
    cv2.addWeighted(overlay, 0.78, frame, 0.22, 0, frame)
    cv2.putText(frame, "GIMBAL // TRACKING", (26, 38),
                cv2.FONT_HERSHEY_DUPLEX, 0.55, white, 1, cv2.LINE_AA)
    cv2.putText(frame, f"STATUS  {status}",
                (26, 63), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                accent if marker_array is not None else warning, 1, cv2.LINE_AA)
    cv2.putText(frame, f"RESPONSE {response_ms:6.1f} ms", (26, 86),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, white, 1, cv2.LINE_AA)
    cv2.putText(frame, f"FPS      {fps:6.1f}", (26, 108),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, white, 1, cv2.LINE_AA)

    if marker_array is not None and servo_result is not None:
        _, _, yaw, pitch, distance = servo_result
        cv2.putText(frame, f"RANGE {distance:.2f}m  PAN {yaw:+.1f}  TILT {pitch:+.1f}",
                    (18, height - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                    white, 1, cv2.LINE_AA)
    else:
        cv2.putText(frame, "Q  EXIT", (18, height - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, white, 1, cv2.LINE_AA)


class SimulinkPoseStreamer:
    """Send [x, y, z, pitch, yaw, distance] pose packets over UDP."""

    def __init__(self, host=SIMULINK_HOST, port=SIMULINK_PORT):
        self.address = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sequence = 0
        self.last_send_time = None
        self.packet_interval_ms = None
        self.interval_total_ms = 0.0
        self.interval_count = 0
        self.cumulative_average_ms = None

    def send(self, marker_array):
        if marker_array is None:
            return None, None
        if len(marker_array) != 6:
            raise ValueError("marker_array must contain six pose values")
        packet = struct.pack("<6dI", *marker_array, self.sequence)
        self.socket.sendto(packet, self.address)
        send_time = time.perf_counter()
        if self.last_send_time is not None:
            self.packet_interval_ms = (send_time - self.last_send_time) * 1000.0
            self.interval_total_ms += self.packet_interval_ms
            self.interval_count += 1
            self.cumulative_average_ms = self.interval_total_ms / self.interval_count
        self.last_send_time = send_time
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        return self.packet_interval_ms, self.cumulative_average_ms

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
    origin_pose = None
    coordinates_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "coordinates.csv")
    coordinates_file = open(coordinates_path, "w", newline="")
    coordinates_writer = csv.writer(coordinates_file)
    coordinates_writer.writerow([
        "timestamp", "raw_x_m", "raw_y_m", "raw_z_m",
        "relative_x_m", "relative_y_m", "relative_z_m",
        "relative_yaw_deg", "relative_pitch_deg",
    ])
    coordinates_file.flush()
    origin_locked = False
    origin_angles = gimbal.current_angles
    search_yaw_direction = 1.0
    search_pitch_direction = -1.0

    try:
        while True:
            frame_start = time.perf_counter()
            frame = picam2.capture_array()

            if not origin_locked:
                origin_center, corners, ids = detect_marker_center(
                    frame, detector, dictionary, parameters, legacy_api,
                    ORIGIN_MARKER_ID,
                )
                if origin_center is None:
                    _, _, hit_yaw_limit, hit_pitch_limit = gimbal.search_step(
                        search_yaw_direction, search_pitch_direction
                    )
                    if hit_yaw_limit:
                        search_yaw_direction *= -1.0
                    if hit_pitch_limit:
                        search_pitch_direction *= -1.0
                    marker_array = None
                    servo_result = None
                    status = "SEARCHING FOR ORIGIN ID 0"
                else:
                    center_x, center_y = origin_center
                    pixel_error_x = center_x - CX
                    pixel_error_y = center_y - CY
                    if (
                        abs(pixel_error_x) <= ORIGIN_CENTER_TOLERANCE_PX
                        and abs(pixel_error_y) <= ORIGIN_CENTER_TOLERANCE_PX
                    ):
                        origin_pose, _, _ = detect_marker_pose(
                            frame, detector, dictionary, parameters, legacy_api,
                            required_marker_id=ORIGIN_MARKER_ID,
                        )
                        if origin_pose is not None:
                            origin_locked = True
                            origin_angles = gimbal.current_angles
                            gimbal.reset_control_state()
                            marker_array = None
                            servo_result = None
                            status = "ORIGIN LOCKED"
                            print(
                                f"Origin locked on marker ID {ORIGIN_MARKER_ID}: "
                                f"x={origin_pose[0]:+.3f}m, y={origin_pose[1]:+.3f}m, "
                                f"z={origin_pose[2]:+.3f}m, "
                                f"yaw={origin_angles[0]:+.2f} deg, "
                                f"pitch={origin_angles[1]:+.2f} deg"
                            )
                        else:
                            marker_array = None
                            servo_result = None
                            status = "ORIGIN POSE UNAVAILABLE"
                    else:
                        yaw_error = -math.degrees(math.atan2(pixel_error_x, FX))
                        pitch_error = math.degrees(math.atan2(pixel_error_y, FY))
                        gimbal.update_direction(yaw_error, pitch_error)
                        marker_array = None
                        servo_result = None
                        status = "CENTERING ORIGIN ID 0"
            else:
                marker_array, target_rotation, corners, ids = detect_marker_pose(
                    frame, detector, dictionary, parameters, legacy_api,
                    excluded_marker_id=ORIGIN_MARKER_ID,
                    return_rotation=True,
                )
                origin_marker, origin_rotation, _, _ = detect_marker_pose(
                    frame, detector, dictionary, parameters, legacy_api,
                    required_marker_id=ORIGIN_MARKER_ID,
                    return_rotation=True,
                )
                camera_angles = gimbal.current_angles
                if marker_array is not None and origin_marker is not None:
                    relative_pose = pose_relative_to_marker(
                        marker_array, origin_marker, origin_rotation
                    )
                elif marker_array is not None:
                    relative_pose = pose_relative_to_origin(
                        marker_array, origin_angles, camera_angles, origin_pose
                    )
                else:
                    relative_pose = None
                packet_interval_ms, cumulative_average_ms = simulink.send(relative_pose)
                if relative_pose is not None:
                    relative_yaw_deg = math.degrees(
                        math.atan2(-relative_pose[0], relative_pose[2])
                    )
                    relative_pitch_deg = math.degrees(
                        math.atan2(
                            relative_pose[1],
                            math.hypot(relative_pose[0], relative_pose[2]),
                        )
                    )
                    coordinates_writer.writerow([
                        f"{time.time():.6f}",
                        f"{marker_array[0]:.6f}", f"{marker_array[1]:.6f}",
                        f"{marker_array[2]:.6f}",
                        f"{relative_pose[0]:.6f}", f"{relative_pose[1]:.6f}",
                        f"{relative_pose[2]:.6f}",
                        f"{relative_yaw_deg:.6f}", f"{relative_pitch_deg:.6f}",
                    ])
                    coordinates_file.flush()
                servo_result = process_marker(marker_array, gimbal, origin_angles)
                status = "TRACKING"

                if packet_interval_ms is not None:
                    print(
                        f"Simulink packet interval: {packet_interval_ms:.2f} ms | "
                        f"cumulative average: {cumulative_average_ms:.2f} ms"
                    )

            response_ms = (time.perf_counter() - frame_start) * 1000.0
            fps = 1000.0 / response_ms if response_ms > 0 else 0.0

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
            draw_target_hud(frame, marker_array, servo_result, response_ms, fps, status)
            cv2.imshow("Gimbal servo control", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        gimbal.stop()
        simulink.close()
        coordinates_file.close()
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()