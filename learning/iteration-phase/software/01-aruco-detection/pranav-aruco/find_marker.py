import cv2
import numpy as np
from pose_detector import pose_detector
import time

marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

param_markers = cv2.aruco.DetectorParameters()


cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    #start timer
    start_time = time.perf_counter()

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #to grayscale
    detector = cv2.aruco.ArucoDetector(
         marker_dict, param_markers
    )
    marker_corners, marker_IDs, reject = detector.detectMarkers(gray_frame)

    if marker_corners:
        for ids, corners in zip(marker_IDs, marker_corners):
            cv2.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
            )

            #order the corners(TL,TR,BR,BL)
            corners_2d = corners.reshape(4, 2)
            corners = corners_2d.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()

            cv2.putText(
                frame,
                f"id: {ids[0]}",
                top_right,
                cv2.FONT_HERSHEY_PLAIN,
                1.3,
                (200, 100, 0),
                2,
                cv2.LINE_AA,
            )
            # print(ids, "  ", corners)

            pose_detector(corners_2d,frame)
    
        #end timer
        end_time = time.perf_counter()

        inference_time = (end_time - start_time)*1000

        print(f"Inference time: {inference_time:.5f}\n")  #printing only when marker is found
    cv2.imshow("frame", frame)
    key = cv2.waitKey(1)
    if key == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
