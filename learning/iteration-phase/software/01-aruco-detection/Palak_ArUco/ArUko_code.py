import cv2
import numpy as np
import time

# Settings
CAM_INDEX = 0
MARKER_LENGTH = 0.05  # meters; set to the real marker side length
DICT_ID = cv2.aruco.DICT_4X4_50


# ArUco detector setup
aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)
params = cv2.aruco.DetectorParameters()


# Speed-focused params
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
params.adaptiveThreshWinSizeMin = 3
params.adaptiveThreshWinSizeMax = 23
params.adaptiveThreshWinSizeStep = 10
params.minMarkerPerimeterRate = 0.03
params.maxMarkerPerimeterRate = 4.0

detector = cv2.aruco.ArucoDetector(aruco_dict, params)


# Webcam open
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
if not cap.isOpened():
    raise RuntimeError("Could not open webcam. Try CAM_INDEX = 1 or close other camera apps.")

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

ret, frame = cap.read()
if not ret:
    cap.release()
    raise RuntimeError("Could not read from webcam. Check camera permissions and backend.")

h, w = frame.shape[:2]
focal = float(w)
camera_matrix = np.array([
    [focal, 0, w / 2.0],
    [0, focal, h / 2.0],
    [0, 0, 1.0]
], dtype=np.float64)
dist_coeffs = np.zeros((5, 1), dtype=np.float64)

print("Using approximate camera matrix:")
print(camera_matrix)
print("Press q to quit.")


# Object points for a square marker centered at origin
obj_points = np.array([
    [-MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0.0],
    [ MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0.0],
    [ MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0.0],
    [-MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0.0]
], dtype=np.float64)

def estimate_pose(corner):
    img_points = corner.reshape(4, 2).astype(np.float64)
    success, rvec, tvec = cv2.solvePnP(
        obj_points,
        img_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    return success, rvec, tvec


# Main loop
while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame grab failed.")
        break

    t0 = time.perf_counter()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    inference_ms = (time.perf_counter() - t0) * 1000.0

    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for i, marker_id in enumerate(ids.flatten()):
            success, rvec, tvec = estimate_pose(corners[i])

            if success:
                cv2.drawFrameAxes(
                    frame,
                    camera_matrix,
                    dist_coeffs,
                    rvec,
                    tvec,
                    MARKER_LENGTH * 0.5
                )

                x, y, z = tvec.flatten()
                print(
                    f"ID={marker_id} | "
                    f"x={x:.4f} m, y={y:.4f} m, z={z:.4f} m | "
                    f"inference={inference_ms:.2f} ms"
                )

                c = corners[i][0]
                cx = int(np.mean(c[:, 0]))
                cy = int(np.mean(c[:, 1]))
                cv2.putText(
                    frame,
                    f"ID {marker_id}",
                    (cx - 20, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA
                )

    cv2.putText(
        frame,
        f"Inference: {inference_ms:.2f} ms",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA
    )

    cv2.imshow("ArUco Pose Estimation", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
