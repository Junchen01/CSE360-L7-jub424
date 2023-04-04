### this code works only on the raspberry
import numpy as np
import cv2
import time
import imutils
import math
from picamera2 import Picamera2
import sys
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

positions = {}
rotations = {}


picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

def get_coordinate(u_d, s_b):
    theta = -0.0875 * u_d + 117.9
    distance = -0.675 * math.log(s_b) + 2.9205
    x = distance * math.cos(theta)
    y = distance * math.sin(theta)
    coordinate = [x, y]
    return coordinate

def coordinate_transformation(sub_coordinate, robot_coordinate, robot_rotation):
    
    xa = robot_coordinate[0]
    ya = robot_coordinate[1]
    xb = sub_coordinate[0]
    yb = sub_coordinate[1]

    # find transformation matrix
    theta_a = np.deg2rad(robot_rotation)
    T = np.array([[np.cos(theta_a), -np.sin(theta_a), xa],
              [np.sin(theta_a), np.cos(theta_a), ya],
              [0, 0, 1]])
    # Vector representing the coordinates of object B in the coordinate system of object A
    v = np.array([[xb], [yb], [1]])
    # Multiply transformation matrix T by vector v to obtain coordinates of object B in the world coordinate system
    w = T @ v
    # Extract x and y coordinates of object B in the world coordinate system
    x = w[0][0]
    y = w[1][0] 
    return [x, y]

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz

if __name__ == "__main__":
    clientAddress = "192.168.0.204"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 204

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()

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
    if radius is not None and radius > 15:
        # print("coordinate: " + str(get_coordinate(center[0], radius)))
        # print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
        # print("coordinate_transformation: " + str(coordinate_transformation(get_coordinate(center[0], radius), positions[robot_id], rotations[robot_id])))
        cor = coordinate_transformation(get_coordinate(center[0], radius), positions[robot_id], rotations[robot_id])
        output_string = "{:.2f} {:.2f}".format(cor[0], cor[1])
        print(output_string)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    time.sleep(0.5)

# When everything done, release the capture
cv2.destroyAllWindows()


