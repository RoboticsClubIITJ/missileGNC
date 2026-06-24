import cv2
import numpy as np
import os
import time

# 1. UPDATED PHYSICAL MARKER SIZE (5.7 cm converted to meters)
MARKER_SIZE_METERS = 0.057  

script_dir = os.path.dirname(os.path.abspath(__file__))
calib_path = os.path.join(script_dir, 'calibration.npz')

# Load calibration profile
try:
    calib_data = np.load(calib_path)
    cam_matrix = calib_data['mtx'].copy()
    dist_coeffs = calib_data['dist']
    print(f"Successfully loaded {calib_path}!")
except FileNotFoundError:
    print("Error: 'calibration.npz' not found! Run calibration script first.")
    exit()

# Setup detector with screen tweaks for high sensitivity
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters()
aruco_params.adaptiveThreshWinSizeMin = 3
aruco_params.adaptiveThreshWinSizeMax = 23
aruco_params.adaptiveThreshWinSizeStep = 10
aruco_params.polygonalApproxAccuracyRate = 0.03
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

marker_3d_edges = np.array([
    [-MARKER_SIZE_METERS / 2,  MARKER_SIZE_METERS / 2, 0],
    [ MARKER_SIZE_METERS / 2,  MARKER_SIZE_METERS / 2, 0],
    [ MARKER_SIZE_METERS / 2, -MARKER_SIZE_METERS / 2, 0],
    [-MARKER_SIZE_METERS / 2, -MARKER_SIZE_METERS / 2, 0]
], dtype=np.float32)

cap = cv2.VideoCapture(0)

# Get stream width dimension to correctly adjust calibration matrices
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))

# --- ADJUST CALIBRATION FOR THE INCOMING FLIPPED MATH ---
cam_matrix[0, 2] = frame_width - cam_matrix[0, 2]

# Create and configure full-screen window
window_name = 'ASUS TUF - Telemetry Dashboard'
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

print("\n--- Tracking Active (Incoming Feed Flipped Internally) ---")
print("Press 'Q' or 'ESC' to exit.")

while True:
    start_time = time.perf_counter()

    ret, raw_frame = cap.read()
    if not ret:
        break

    # 1. FLIP AND FORCE A CLEAN MEMORY COPY
    frame = cv2.flip(raw_frame, 1).copy()

    # 2. RUN DETECTOR ON THE CLEAN FLIPPED MATRIX
    corners, ids, rejected = detector.detectMarkers(frame)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(ids)):
            success, rvec, tvec = cv2.solvePnP(
                marker_3d_edges, corners[i][0], cam_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
            )

            if success:
                # Draw 3D Axes directly over the flipped framespace
                cv2.drawFrameAxes(frame, cam_matrix, dist_coeffs, rvec, tvec, MARKER_SIZE_METERS * 1.5)

                # Calculate Position Data (in cm)
                x_cm = tvec[0][0] * 100
                y_cm = tvec[1][0] * 100
                z_cm = tvec[2][0] * 100 

                # Calculate Orientation Data (in degrees)
                rmat, _ = cv2.Rodrigues(rvec) 
                pitch = np.arctan2(rmat[2, 1], rmat[2, 2]) * 180 / np.pi
                yaw = np.arctan2(-rmat[2, 0], np.sqrt(rmat[2, 1]**2 + rmat[2, 2]**2)) * 180 / np.pi
                roll = np.arctan2(rmat[1, 0], rmat[0, 0]) * 180 / np.pi

                # Render Data Text Strings safely
                text_pos = f"Position X:{x_cm:.1f} Y:{y_cm:.1f} Z:{z_cm:.1f} cm"
                text_ori = f"Orientation P:{pitch:.1f} Y:{yaw:.1f} R:{roll:.1f} deg"
                
                cv2.putText(frame, text_pos, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
                cv2.putText(frame, text_pos, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                cv2.putText(frame, text_ori, (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
                cv2.putText(frame, text_ori, (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    # Calculate Total Pipeline Latency
    end_time = time.perf_counter()
    inference_time_ms = (end_time - start_time) * 1000
    
    text_inf = f"Inference Time: {inference_time_ms:.1f} ms"
    cv2.putText(frame, text_inf, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
    cv2.putText(frame, text_inf, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Display the final frame composition
    cv2.imshow(window_name, frame)

    # Keyboard checks
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:
        break

cap.release()
cv2.destroyAllWindows()
