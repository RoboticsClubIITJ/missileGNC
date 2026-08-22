import cv2
import numpy as np
import time
from picamera2 import Picamera2

# =======================================================
# MUST MATCH generate_charuco.py exactly
# =======================================================
SQUARES_X = 7
SQUARES_Y = 10
SQUARE_LENGTH_M = 0.0105   # <-- measure your PRINTED board's square edge and update this
MARKER_LENGTH_M = 0.007  # <-- measure your PRINTED board's marker edge and update this

WIDTH, HEIGHT = 640, 480
MIN_CAPTURES = 15  # aim for 15-25 good captures from varied angles/distances

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard(
    (SQUARES_X, SQUARES_Y), SQUARE_LENGTH_M, MARKER_LENGTH_M, dictionary
)
detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
charuco_detector = cv2.aruco.CharucoDetector(board)

picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (WIDTH, HEIGHT), "format": "RGB888"})
picam2.configure(config)
picam2.start()
time.sleep(2)

all_charuco_corners = []
all_charuco_ids = []
captured = 0

print("Move the board around (angle, distance, position in frame).")
print("Press SPACE to capture a good detection, 'q' when done (need >=", MIN_CAPTURES, ")")

try:
    while True:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(gray)

        display = frame.copy()
        if marker_ids is not None and len(marker_ids) > 0:
            cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)
        if charuco_ids is not None and charuco_corners is not None and len(charuco_ids) > 0:
            try:
                cv2.aruco.drawDetectedCornersCharuco(display, charuco_corners, charuco_ids)
            except cv2.error:
                pass  # shape mismatch on this frame, skip drawing, keep going

        cv2.putText(display, f"Captured: {captured}/{MIN_CAPTURES}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Calibration Capture", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            valid = False
            if charuco_ids is not None and charuco_corners is not None and len(charuco_ids) >= 6:
                try:
                    valid = (len(charuco_corners) == len(charuco_ids))
                except Exception:
                    valid = False
            if valid:
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
                captured += 1
                print(f"Captured {captured} (this frame had {len(charuco_ids)} corners)")
            else:
                print("Not enough corners visible, reposition and try again")
        elif key == ord('q'):
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()

if captured < 4:
    print("Not enough captures to calibrate. Need at least 4, ideally 15+.")
    exit(1)

print(f"\nRunning calibration on {captured} captures...")

# cv2.aruco.calibrateCameraCharuco was removed in newer OpenCV (4.7+).
# Use the version-proof path: board.matchImagePoints() + cv2.calibrateCamera().
all_obj_points = []
all_img_points = []
for corners, ids in zip(all_charuco_corners, all_charuco_ids):
    obj_points, img_points = board.matchImagePoints(corners, ids)
    if obj_points is not None and img_points is not None and len(obj_points) >= 4:
        all_obj_points.append(obj_points)
        all_img_points.append(img_points)

if len(all_obj_points) < 4:
    print("Not enough valid captures survived point-matching. Re-run and capture more varied angles.")
    exit(1)

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    all_obj_points, all_img_points, (WIDTH, HEIGHT), None, None
)

print("\n=======================================================")
print(f"Calibration RMS reprojection error: {ret:.4f}  (lower is better, <1.0 is good)")
print("\ncamera_matrix = np.array([")
for row in camera_matrix:
    print(f"    [{row[0]:.4f}, {row[1]:.4f}, {row[2]:.4f}],")
print("], dtype=np.float32)")
print("\ndist_coeffs = np.array([", ", ".join(f"{c:.6f}" for c in dist_coeffs.flatten()), "], dtype=np.float32)")
print("=======================================================")

np.savez("/home/rocket/camera_calibration.npz",
         camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, rms_error=ret)
print("\nSaved to ~/camera_calibration.npz")
print("Copy the camera_matrix / dist_coeffs values above into your ArUco detection script.")