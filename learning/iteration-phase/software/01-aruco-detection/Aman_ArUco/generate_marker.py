import cv2

# Load the standard 6x6 grid dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Generate Marker ID 24 (size: 400x400 pixels)
# 'img' will hold the image matrix data
img = cv2.aruco.generateImageMarker(aruco_dict, 24, 400)

# Save the matrix data as a PNG image file
cv2.imwrite("my_marker.png", img)
print("Success! 'my_marker.png' has been saved to your project folder.")
