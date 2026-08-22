import cv2
import numpy as np
import math
import os
import time
from picamera2 import Picamera2

# Force OpenCV to utilize all 4 Raspberry Pi CPU cores for operations
cv2.setNumThreads(4)

def main():
    # Set the physical size of your printed ArUco marker in meters
    MARKER_SIZE = 0.05
    ADAPTIVE_THRESH_WINDOWS = 1
    DETECTION_SCALE = 0.75

    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "OpenCV was built without ArUco support. Install an opencv-contrib build."
        )

    # =======================================================
    # RASPBERRY PI CAMERA V1.3 CONFIGURATION (via Picamera2/libcamera)
    # =======================================================
    # On Ubuntu 24.04, cv2.VideoCapture(0) reads the raw Unicam CSI
    # device (Bayer data), which OpenCV can't decode correctly. Picamera2
    # goes through libcamera + the ISP and hands back a normal BGR frame,
    # same as picamera2 does on Raspberry Pi OS.
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    # Let auto-exposure/auto-white-balance settle
    time.sleep(2)

    # HARDCODED INTRINSICS (MUST BE CALIBRATED FOR TRUE ACCURACY)
    # Placeholder for Pi Cam v1.3 (OV5647) at 640x480
    focal_length = 500.0
    center_x = 320.0
    center_y = 240.0

    camera_matrix = np.array([
        [focal_length, 0.0, center_x],
        [0.0, focal_length, center_y],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)

    dist_coeffs = np.zeros((5, 1), dtype=np.float32)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calibration_file = next(
        (
            path
            for path in (
                os.path.join(script_dir, "camera_calibration.npz"),
                os.path.join(script_dir, "calibration.npz"),
            )
            if os.path.exists(path)
        ),
        None,
    )
    if calibration_file is not None:
        with np.load(calibration_file) as calibration:
            camera_matrix_key = "camera_matrix" if "camera_matrix" in calibration else "mtx"
            distortion_key = "dist_coeffs" if "dist_coeffs" in calibration else "dist"
            camera_matrix = calibration[camera_matrix_key].astype(np.float32)
            dist_coeffs = calibration[distortion_key].reshape(-1, 1).astype(np.float32)

        if (
            camera_matrix.shape != (3, 3)
            or dist_coeffs.shape[0] < 4
            or not np.isfinite(camera_matrix).all()
            or not np.isfinite(dist_coeffs).all()
        ):
            raise ValueError(f"Invalid calibration data in {calibration_file}")

        print(f"Loaded camera calibration from {calibration_file}")
    # =======================================================

    # Setup ArUco Dictionary and Parameters with optimizations
    if hasattr(cv2.aruco, 'ArucoDetector'):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        parameters.adaptiveThreshWinSizeMin = 5
        parameters.adaptiveThreshWinSizeMax = 5 + (ADAPTIVE_THRESH_WINDOWS - 1) * 5
        parameters.adaptiveThreshWinSizeStep = 5
        if hasattr(cv2.aruco, "CORNER_REFINE_NONE"):
            parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        legacy_api = False
    else:
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.adaptiveThreshWinSizeMin = 5
        parameters.adaptiveThreshWinSizeMax = 5 + (ADAPTIVE_THRESH_WINDOWS - 1) * 5
        parameters.adaptiveThreshWinSizeStep = 5
        if hasattr(cv2.aruco, "CORNER_REFINE_NONE"):
            parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        legacy_api = True

    marker_points = np.array([
        [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
        [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
        [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
        [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]
    ], dtype=np.float32)

    try:
        while True:
            # Capture frame-by-frame from the Pi Camera via libcamera
            frame = picam2.capture_array()  # RGB888 -> actually BGR-ordered array from Picamera2

            # Convert to grayscale for the detector
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # =======================================================
            # STRICT INFERENCE TIMING BLOCK
            # =======================================================
            start_time = time.perf_counter()
            detection_gray = cv2.resize(
                gray, None, fx=DETECTION_SCALE, fy=DETECTION_SCALE,
                interpolation=cv2.INTER_AREA
            )

            if not legacy_api:
                corners, ids, rejected = detector.detectMarkers(detection_gray)
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    detection_gray, dictionary, parameters=parameters
                )

            # Keep pose estimation in the original calibrated camera coordinate system.
            if corners:
                corners = [corner / DETECTION_SCALE for corner in corners]
                refinement_criteria = (
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30,
                    0.001,
                )
                corners = [
                    cv2.cornerSubPix(
                        gray,
                        corner.astype(np.float32),
                        (5, 5),
                        (-1, -1),
                        refinement_criteria,
                    )
                    for corner in corners
                ]

            marker_array = None  # will hold [x, y, z, pitch, yaw] for the detected marker
            pose_results = []

            if ids is not None and len(ids) > 0:
                for i, corner in enumerate(corners):
                    # IPPE_SQUARE is the fastest PnP solver for 4 coplanar points
                    success, rvec, tvec = cv2.solvePnP(
                        marker_points, corner[0], camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    if not success:
                        continue

                    t = tvec.flatten()
                    x, y, z = t[0], t[1], t[2]
                    distance = math.sqrt(x**2 + y**2 + z**2)

                    rot_mat, _ = cv2.Rodrigues(rvec)
                    pitch, yaw, roll = cv2.RQDecomp3x3(rot_mat)[0]

                    # [x, y, z] in meters (camera frame), pitch/yaw in degrees,
                    # distance in meters (straight-line from camera to marker).
                    # Overwritten each loop -> holds the LAST detected marker in
                    # this frame if multiple markers are visible at once.
                    marker_array = [float(x), float(y), float(z), float(pitch), float(yaw), float(distance)]
                    pose_results.append((i, rvec, tvec, x, y, z, pitch, yaw, roll))

            end_time = time.perf_counter()
            # =======================================================

            inference_time_ms = (end_time - start_time) * 1000
            fps = 1.0 / (end_time - start_time) if (end_time - start_time) > 0 else 0.0

            # Copy original frame for drawing visual elements
            display_frame = frame.copy()

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)
                for i, rvec, tvec, x, y, z, pitch, yaw, roll in pose_results:
                    cv2.drawFrameAxes(display_frame, camera_matrix, dist_coeffs, rvec, tvec, MARKER_SIZE / 2)

                    print(f"Marker ID: {ids.flatten()[i]}")
                    print(f"  Coordinates : X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m")
                    print(f"  Rotation    : P={pitch:.1f}°, Y={yaw:.1f}°, R={roll:.1f}°")

            if marker_array is not None:
                # This is the array you asked for: [x, y, z, pitch, yaw, distance]
                print(f"  Array       : {marker_array}")
                # marker_array is a plain Python list here — feed it directly
                # into a serial/socket send, a queue, or a gimbal controller
                # call at this point if/when you wire that up.

            print(f"  Inference   : {inference_time_ms:.2f} ms | FPS: {fps:.2f}")
            print("-" * 55)

            # Draw metrics directly onto the video frame
            cv2.putText(display_frame, f"Inference: {inference_time_ms:.2f} ms", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, f"FPS: {fps:.2f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Display the live video feed
            cv2.imshow("Raspberry Pi Cam ArUco Detection", display_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        # Safely release the hardware lock on the Pi camera when closing the script
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
