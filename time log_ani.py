import cv2
import time

# Initialize camera
cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Initialize ArUco
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector_parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, detector_parameters)

print("Testing latency... Press 'q' to quit.")

while True:
    # 1. Measure Camera Capture Latency
    start_capture = time.perf_counter()
    ret, frame = cap.read()
    end_capture = time.perf_counter()
    
    if not ret:
        break
        
    capture_delay = (end_capture - start_capture) * 1000 # Convert to milliseconds

    # 2. Measure ArUco Processing Latency
    start_processing = time.perf_counter()
    corners, ids, rejected = detector.detectMarkers(frame)
    end_processing = time.perf_counter()
    
    processing_delay = (end_processing - start_processing) * 1000 # Convert to milliseconds
    
    # Total Latency for this loop cycle
    total_delay = capture_delay + processing_delay
    fps = 1000 / total_delay if total_delay > 0 else 0

    # Draw markers if found
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # 3. Overlay the latency metrics onto the video frame
    cv2.putText(frame, f"Cam Grab: {capture_delay:.1f} ms", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(frame, f"ArUco Proc: {processing_delay:.1f} ms", (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(frame, f"Total Delay: {total_delay:.1f} ms ({int(fps)} FPS)", (10, 90), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    cv2.imshow("Latency Benchmark", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()