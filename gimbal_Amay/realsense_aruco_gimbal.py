import cv2
import numpy as np
import pyrealsense2 as rs
import math
import time

# Force OpenCV to utilize all 4 Raspberry Pi CPU cores for operations
cv2.setNumThreads(4)

def main():
    # Set the physical size of your printed ArUco marker in meters
    MARKER_SIZE = 0.05 

    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Standard configuration supported by most RealSense cameras over USB 3.0/2.0
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)

    # Start RealSense camera
    profile = pipeline.start(config)

    # Get camera intrinsics directly from the RealSense hardware
    color_stream = profile.get_stream(rs.stream.color)
    intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
    
    camera_matrix = np.array([
        [intrinsics.fx, 0, intrinsics.ppx],
        [0, intrinsics.fy, intrinsics.ppy],
        [0, 0, 1]
    ], dtype=np.float32)
    dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float32)

    # Setup ArUco Dictionary and Parameters with optimizations
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        parameters.adaptiveThreshWinSizeMin = 5
        parameters.adaptiveThreshWinSizeMax = 15
        parameters.adaptiveThreshWinSizeStep = 5
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        legacy_api = False
    else:
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.adaptiveThreshWinSizeMin = 5
        parameters.adaptiveThreshWinSizeMax = 15
        parameters.adaptiveThreshWinSizeStep = 5
        legacy_api = True

    marker_points = np.array([
        [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
        [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
        [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
        [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]
    ], dtype=np.float32)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Standard color frame extraction and grayscale conversion
            frame = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # =======================================================
            # STRICT INFERENCE TIMING BLOCK
            # =======================================================
            start_time = time.perf_counter()

            if not legacy_api:
                corners, ids, rejected = detector.detectMarkers(gray)
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

            pitch, yaw, roll, x, y, z, distance = 0, 0, 0, 0, 0, 0, 0
            
            if ids is not None and len(ids) > 0:
                for i, corner in enumerate(corners):
                    # IPPE_SQUARE is the fastest PnP solver for 4 coplanar points
                    _, rvec, tvec = cv2.solvePnP(
                        marker_points, corner, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    
                    t = tvec.flatten()
                    x, y, z = t[0], t[1], t[2]
                    distance = math.sqrt(x**2 + y**2 + z**2)
                    
                    rot_mat, _ = cv2.Rodrigues(rvec)
                    proj_mat = np.hstack((rot_mat, tvec))
                    euler_angles = cv2.decomposeProjectionMatrix(proj_mat)[6]
                    pitch, yaw, roll = euler_angles.flatten()

            end_time = time.perf_counter()
            # =======================================================
            
            inference_time_ms = (end_time - start_time) * 1000
            fps = 1.0 / (end_time - start_time) if (end_time - start_time) > 0 else 0.0
            
            # Copy original frame for drawing visual elements
            display_frame = frame.copy()

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)
                for i, corner in enumerate(corners):
                    cv2.drawFrameAxes(display_frame, camera_matrix, dist_coeffs, rvec, tvec, MARKER_SIZE / 2)
                    
                    print(f"Marker ID: {ids.flatten()[i]}")
                    print(f"  Coordinates : X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m")
                    print(f"  Rotation    : P={pitch:.1f}°, Y={yaw:.1f}°, R={roll:.1f}°")

            print(f"  Inference   : {inference_time_ms:.2f} ms | FPS: {fps:.2f}")
            print("-" * 55)

            # Draw metrics directly onto the video frame
            cv2.putText(display_frame, f"Inference: {inference_time_ms:.2f} ms", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, f"FPS: {fps:.2f}", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Display the live video feed
            cv2.imshow("RealSense ArUco Detection", display_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        pipeline.stop()

if __name__ == "__main__":
    main()
