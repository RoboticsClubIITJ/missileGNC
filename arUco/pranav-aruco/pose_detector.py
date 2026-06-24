import cv2
import numpy as np

try:
    # Load the NumPy arrays directly from the files
    camera_matrix = np.load('camera_calibration/calib_data/camMatrix.npy')
    dist_coeffs = np.load('camera_calibration/calib_data/distCoef.npy')
    print("Camera calibration data loaded successfully!\n")
except FileNotFoundError:
    print("Error: Calibration files not found. Ensure they are in the same folder.")
    exit()



#marker size
MARKER_SIZE = 0.055

marker_3d_edges = np.array([
    [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0], # Top-Left
    [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0], # Top-Right
    [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0], # Bottom-Right
    [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]  # Bottom-Left
], dtype=np.float32)

def pose_detector(corners_2d,frame):
    success, rvec, tvec = cv2.solvePnP(
                    marker_3d_edges, 
                    corners_2d, 
                    camera_matrix, 
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
    
    if success:
        #draw lines
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, MARKER_SIZE / 2)

        #calc rotation(pitch,roll,yaw)
        rmat,_ = cv2.Rodrigues(rvec)
        euler_angles,_,_,_,_,_ = cv2.RQDecomp3x3(rmat)
        pitch = euler_angles[0]
        roll = euler_angles[1]
        yaw = euler_angles[2]

        #calc coordinates
        x = tvec[0][0]
        y = tvec[1][0]
        z = tvec[2][0]

        print(f"Pitch: {pitch:.2f}, Roll: {roll:.2f}, Yaw: {yaw:.2f}")
        print(f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}")
