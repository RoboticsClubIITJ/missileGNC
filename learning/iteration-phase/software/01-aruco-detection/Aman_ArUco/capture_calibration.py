import cv2
import os

os.makedirs('calib_images', exist_ok=True)
cap = cv2.VideoCapture(0)
count = 0

print("Hold your checkerboard flat in front of the webcam.")
print("Press 'C' to snap a photo | Press 'Q' to finish.")

while True:
    ret, frame = cap.read()
    if not ret: break

    preview = frame.copy()
    cv2.putText(preview, f'Photos Saved: {count}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow('Calibration Window', preview)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'): break
    elif key == ord('c'):
        cv2.imwrite(f'calib_images/frame_{count:02d}.jpg', frame)
        count += 1
        print(f"Captured photo {count}")

cap.release()
cv2.destroyAllWindows()
