import socket
import time
import sys
import time
import math
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

positions = {}
rotations = {}
target = [3.13,1.94]
IP_ADDRESS = '192.168.0.212'
clientAddress = "192.168.0.40"
optitrackServerAddress = "192.168.0.4"
robot_id = 212
v = 0
omega = 0
distance = 10000

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz



# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')
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

time_index = 0
try:
    while is_running:
        if robot_id in positions:
            # last position
            print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
            
            print('-----------------------------')
            
           
            # send_speed(v, omega)
            command = 'CMD_MOTOR#100#100#1200#1200\n'
            s.send(command.encode('utf-8'))

            time.sleep(0.1)

            
except KeyboardInterrupt:
    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))

# Close the connection
command = 'CMD_MOTOR#00#00#00#00\n'
s.send(command.encode('utf-8'))
s.shutdown(2)
s.close()
streaming_client.shutdown()