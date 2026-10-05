import cv2
import numpy as np
import glob
import os

CHECKERBOARD = (7, 5)  # Inner corner intersections count
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

script_dir = os.path.dirname(os.path.abspath(__file__))
image_dir = os.path.join(script_dir, 'calib_images')
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

obj_points, img_points = [], []
image_shape = None
images = glob.glob(os.path.join(image_dir, '*.jpg'))

if not images:
    raise SystemExit(f"Error: No images found in '{image_dir}'. Put calibration images in that folder.")

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if image_shape is None:
        image_shape = gray.shape[::-1]

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    if ret:
        obj_points.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        img_points.append(corners2)

if not obj_points:
    raise SystemExit(
        f"Error: No chessboard corners found in calibration images. "
        "Check CHECKERBOARD size and verify the images in 'calib_images'."
    )

ret, K, D, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, image_shape, None, None)
output_path = os.path.join(script_dir, 'calibration.npz')
np.savez(output_path, mtx=K, dist=D)
print(f"Success! Matrix calculations written to '{output_path}'")
