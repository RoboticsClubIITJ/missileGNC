import cv2
import numpy as np

# 1. Define the physical marker size in METERS (Crucial for distance math)
# Example: 5 cm = 0.05 meters
MARKER_LENGTH = 0.05 

# 2. Approximate Camera Matrix & Distortion Coefficients 
# (Replace these with your actual calibration data later for millimeter precision)
# Assuming a standard 640x480 resolution lens profile:
focal_length = 640  
center_x = 320
center_y = 240

camera_matrix = np.array([[focal_length, 0, center_x],
                          [0, focal_length, center_y],
                          [0, 0, 1]], dtype=np.float32)

dist_coeffs = np.zeros((5, 1)) # Assuming zero lens distortion for basic approximation

# 3. Setup Video Capture and Detector
cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector_parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, detector_parameters)

# Set up 3D coordinate points of the marker corners in its own local frame
obj_points = np.array([
    [-MARKER_LENGTH / 2.,  MARKER_LENGTH / 2., 0.],
    [ MARKER_LENGTH / 2.,  MARKER_LENGTH / 2., 0.],
    [ MARKER_LENGTH / 2., -MARKER_LENGTH / 2., 0.],
    [-MARKER_LENGTH / 2., -MARKER_LENGTH / 2., 0.]
], dtype=np.float32)

print("Starting Pose Estimation. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, rejected = detector.detectMarkers(frame)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(ids)):
            # 4. Solve PnP (Perspective-n-Point) to calculate orientation and distance
            # rvec = Rotation Vector (Orientation)
            # tvec = Translation Vector (Distance/Position)
            success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

            if success:
                # Draw a 3D coordinate axis onto the marker (Red=X, Green=Y, Blue=Z)
                # The length of the axis lines matches our marker size
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, MARKER_LENGTH)

                # Extract distance along the Z-axis (straight out from the lens)
                # tvec contains [X, Y, Z] spatial coordinates relative to camera center
                distance_meters = tvec[2][0]
                distance_cm = distance_meters * 100

                # Print spatial coordinates to the console
                print(f"Marker ID {ids[i][0]} -> Distance: {distance_cm:.2f} cm | Vector: X={tvec[0][0]:.2f}, Y={tvec[1][0]:.2f}, Z={tvec[2][0]:.2f}")

                # Display distance text over the marker box
                text_coord = tuple(corners[i][0][0].astype(int))
                cv2.putText(frame, f"Dist: {distance_cm:.1f}cm", text_coord, 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("3D Pose Estimation", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()