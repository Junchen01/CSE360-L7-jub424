import cv2

import time

from picamera2 import Picamera2 



# Specify the directory to save the image
save_path = "/home/pi/lab7/img/"


picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()


frame = picam2.capture_array()

# Save the captured frame to the specified directory
cv2.imwrite("img000.jpg", frame)


time.sleep(0.25)

# Release the camera and close all windows
cv2.destroyAllWindows()
