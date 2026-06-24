import cv2

# 1. Initialize the webcam (0 is usually the default built-in camera)
cap = cv2.VideoCapture(1)

# 2. Define the dictionary that matches the markers you generated
# (Must match the dictionary used to create the marker, e.g., 4x4_50)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# 3. Set up the detector parameters and create the detector object
detector_parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, detector_parameters)

print("Press 'q' to quit the video stream.")

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    # 4. Detect the markers in the image
    # corners: List of detected marker corners (x, y coordinates)
    # ids: Array of detected marker IDs
    # rejected: Corners of shapes that looked like markers but failed verification
    corners, ids, rejected = detector.detectMarkers(frame)

    # 5. If markers are found, draw outlines and IDs on the frame
    if ids is not None:
        # Draw green bounding boxes around the detected markers
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Loop through detected markers to print their IDs to the console
        for i in range(len(ids)):
            print(f"Detected Marker ID: {ids[i][0]}")

    # Display the resulting frame
    cv2.imshow("ArUco Detection", frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Clean up and close windows
cap.release()
cv2.destroyAllWindows()
