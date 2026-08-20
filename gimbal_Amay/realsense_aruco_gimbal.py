import cv2
import numpy as np
import math
import time

# Force OpenCV to utilize all 4 Raspberry Pi CPU cores for operations
cv2.setNumThreads(4)

def main():
    # Set the physical size of your printed ArUco marker in meters
    MARKER_SIZE = 0.05 

    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "OpenCV was built without ArUco support. Install an opencv-contrib build."
        )

    # =======================================================
    # RASPBERRY PI CAMERA V1.3 CONFIGURATION
    # =======================================================
    # Initialize standard webcam capture (0 is typically the CSI Pi Camera)
    cap = cv2.VideoCapture(0)
    
    # Force the camera to output a standard 640x480 resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        raise RuntimeError("Error: Could not access the Raspberry Pi Camera.")

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
    
    # Assuming zero distortion until you run a calibration script
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)
    # =======================================================

    # Setup ArUco Dictionary and Parameters with optimizations
    if hasattr(cv2.aruco, 'ArucoDetector'):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        parameters.adaptiveThreshWinSizeMin = 5
        parameters.adaptiveThreshWinSizeMax = 15
        parameters.adaptiveThreshWinSizeStep = 5
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        legacy_api = False
    else:
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.adaptiveThreshWinSizeMin = 5
        parameters.adaptiveThreshWinSizeMax = 15
        parameters.adaptiveThreshWinSizeStep = 5
        legacy_api = True

    marker_points = np.array([
        [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
        [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
        [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
        [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]
    ], dtype=np.float32)

    try:
        while True:
            # Capture frame-by-frame from the Pi Camera
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame. Exiting...")
                break

            # Convert to grayscale for the detector
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # =======================================================
            # STRICT INFERENCE TIMING BLOCK
            # =======================================================
            start_time = time.perf_counter()

            if not legacy_api:
                corners, ids, rejected = detector.detectMarkers(gray)
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

            pitch, yaw, roll, x, y, z, distance = 0, 0, 0, 0, 0, 0, 0
            
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
                    proj_mat = np.hstack((rot_mat, tvec))
                    euler_angles = cv2.decomposeProjectionMatrix(proj_mat)[6]
                    pitch, yaw, roll = euler_angles.flatten()

            end_time = time.perf_counter()
            # =======================================================
            
            inference_time_ms = (end_time - start_time) * 1000
            fps = 1.0 / (end_time - start_time) if (end_time - start_time) > 0 else 0.0
            
            # Copy original frame for drawing visual elements
            display_frame = frame.copy()

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)
                for i, corner in enumerate(corners):
                    success, rvec, tvec = cv2.solvePnP(
                        marker_points, corner[0], camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    if not success:
                        continue

                    cv2.drawFrameAxes(display_frame, camera_matrix, dist_coeffs, rvec, tvec, MARKER_SIZE / 2)
                    
                    print(f"Marker ID: {ids.flatten()[i]}")
                    print(f"  Coordinates : X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m")
                    print(f"  Rotation    : P={pitch:.1f}°, Y={yaw:.1f}°, R={roll:.1f}°")

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
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()