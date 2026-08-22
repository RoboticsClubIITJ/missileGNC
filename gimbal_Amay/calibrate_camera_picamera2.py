import os
import time

import cv2
import numpy as np
from picamera2 import Picamera2


CHECKERBOARD = (9, 6)
SQUARE_SIZE_METERS = 0.024
REQUIRED_SAMPLES = 20
IMAGE_SIZE = (640, 480)
OUTPUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.npz")


def main():
    object_points = np.zeros(
        (CHECKERBOARD[0] * CHECKERBOARD[1], 3), dtype=np.float32
    )
    object_points[:, :2] = np.mgrid[
        0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]
    ].T.reshape(-1, 2)
    object_points *= SQUARE_SIZE_METERS

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    captured_object_points = []
    captured_image_points = []

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": IMAGE_SIZE, "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)

    print(f"Checkerboard: {CHECKERBOARD} inner corners")
    print(f"Square size: {SQUARE_SIZE_METERS:.4f} m")
    print("Move the board around the full image area.")
    print("Press S when corners are visible to capture a sample.")
    print("Press Q to calibrate and quit.")

    try:
        while len(captured_object_points) < REQUIRED_SAMPLES:
            frame = picam2.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray,
                CHECKERBOARD,
                cv2.CALIB_CB_ADAPTIVE_THRESH
                + cv2.CALIB_CB_NORMALIZE_IMAGE
                + cv2.CALIB_CB_FAST_CHECK,
            )

            preview = frame.copy()
            if found:
                refined_corners = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), criteria
                )
                cv2.drawChessboardCorners(
                    preview, CHECKERBOARD, refined_corners, found
                )

            cv2.putText(
                preview,
                f"Samples: {len(captured_object_points)}/{REQUIRED_SAMPLES}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0) if found else (0, 0, 255),
                2,
            )
            cv2.imshow("Camera Calibration", preview)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s") and found:
                captured_object_points.append(object_points.copy())
                captured_image_points.append(refined_corners)
                print(
                    f"Captured sample {len(captured_object_points)} "
                    f"of {REQUIRED_SAMPLES}"
                )

        if len(captured_object_points) < 10:
            raise RuntimeError("Capture at least 10 valid samples before calibrating.")

        image_size = gray.shape[::-1]
        rms_error, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            captured_object_points,
            captured_image_points,
            image_size,
            None,
            None,
        )

        total_error = 0.0
        for object_points_3d, image_points_2d, rvec, tvec in zip(
            captured_object_points,
            captured_image_points,
            rvecs,
            tvecs,
        ):
            projected_points, _ = cv2.projectPoints(
                object_points_3d, rvec, tvec, camera_matrix, dist_coeffs
            )
            total_error += cv2.norm(image_points_2d, projected_points, cv2.NORM_L2) ** 2

        mean_error = np.sqrt(total_error / len(captured_object_points))
        np.savez(
            OUTPUT_FILE,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            mtx=camera_matrix,
            dist=dist_coeffs,
            image_size=np.array(image_size),
            rms_error=rms_error,
            mean_error=mean_error,
        )

        print(f"Saved calibration to: {OUTPUT_FILE}")
        print(f"RMS reprojection error: {rms_error:.4f} pixels")
        print(f"Mean reprojection error: {mean_error:.4f} pixels")
        print("Camera matrix:")
        print(camera_matrix)
        print("Distortion coefficients:")
        print(dist_coeffs.ravel())
    finally:
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
