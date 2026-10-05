import cv2
import os

marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
MARKER_SIZE = 400


output_dir = "markers"
os.makedirs(output_dir, exist_ok=True)


# generating unique IDs markers
for id in range(20):  
    marker_image = cv2.aruco.generateImageMarker(marker_dict, id, MARKER_SIZE)
    cv2.imshow("img", marker_image)
    file_path = os.path.join(output_dir, f"marker_{id}.png")
    cv2.imwrite(file_path, marker_image)

cv2.waitKey(0)
cv2.destroyAllWindows()
