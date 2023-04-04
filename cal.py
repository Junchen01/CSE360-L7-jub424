### this code works only on the raspberry
import numpy as np
import cv2
import time
import imutils

from picamera2 import Picamera2 
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()



while True:
    frame = picam2.capture_array()

    # It converts the BGR color space of image to HSV color space
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

      
    # Threshold of blue in HSV space
    lower_green = np.array([20,100,100])
    upper_green = np.array([30,255,255]) 
  
    # preparing the mask to overlay
    mask = cv2.inRange(hsv, lower_green, upper_green)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    #perform bitwise and on the original image arrays using the mask
    res = cv2.bitwise_and(frame, frame, mask=mask)

    # blur the image to reduce high frequency noise
    res = cv2.medianBlur(mask, 5)

    # detect circles in the image
    rows = res.shape[0]
    #circles = cv2.HoughCircles(res, cv2.HOUGH_GRADIENT, 1, rows / 8,
                            #    param1=100, param2=30,
                            #    minRadius=1, maxRadius=30)
    
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    radius = None
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    
    # Display the resulting frame
    cv2.imshow('frame',res)

    print("rad: " + str(radius))
    print("cen: " + str(center))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    time.sleep(0.05)

# When everything done, release the capture
cv2.destroyAllWindows()


