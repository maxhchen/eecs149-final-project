from djitellopy import Tello
from utils import * 
import cv2, time, sys, math, socket
import cv2.aruco as aruco 
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import csv

# Reference:  https://www.youtube.com/watch?v=wlT_0fhGrGg&t=45s

# Define Tag
marker_size = 20 #cm

# Load Aruco Dictionary 
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters_create() 

# Define the frame size 
width = 1000
height = 800

# Init the drone
drone = init_drone()
font = cv2.FONT_HERSHEY_PLAIN

# From online: 
camera_matrix = np.array([[921.170702, 0.000000, 459.904354], [0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]])
distortion = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])


# Roughly 0.009 seconds per data sampled 
x_values = []
y_values = []
z_values = []
x_vel = []
y_vel = []
z_vel = []
x_error = []
y_error = []
z_error = []
control_fb = []
control_lh= []
control_ud = []
height_values = []
direction_dictionaries = {'x': x_values, 'y': y_values, 'z': z_values, 'x_error': x_error, 'y_error': y_error, 'z_error': z_error, 
'height': height_values, 'vx': x_vel, 'vy': y_vel, 'vz': z_vel, 'fb': control_fb, 'lh': control_lh, 'ud': control_ud}

# To calibrate for take off
# Balloon starts off 2m above ground
print(drone.get_battery())
drone.takeoff()
start_time = time.time()

# PID constants, [P, I, D] --> Only P now 
pX = 0.06
pY = 0.2
pZ = 0.08

# Processing the frame
# For testing, flight time limited to 10s
while True and (time.time() - start_time < 20):

    # Land the drone of the battery is too low
    if drone.get_battery() < 5:
        drone.land()
        print("critical battery: " + drone.get_battery)
        break

    frame = get_tello_frame(drone, width, height)
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)
    if ids != None:  
        x = time.time()
        # ret = [rvec, tvec ?]
        # array of rotation and position of each marker in camera frame
        # rvec = [[rvec_1], [rvec_2]] altitude of the marker respect to marker
        # tvec = [[tvec_1], [tvec_2]] position of the marker respect to marker
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, distortion)
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

        # Collecting Data: 
        direction_dictionaries['x'].append(tvec[0])
        direction_dictionaries['y'].append(tvec[1])
        direction_dictionaries['z'].append(tvec[2])
        direction_dictionaries['height'].append(drone.get_height())
        direction_dictionaries['vx'].append(drone.get_speed_x())
        direction_dictionaries['vy'].append(drone.get_speed_y())
        direction_dictionaries['vz'].append(drone.get_speed_z())

        ####################################################################################################
        # Controller
        center = np.mean(corners[0][0], axis=0)
        xerror = width/2 - center[0]
        yerror = height/2 - center[1]
        disterror = tvec[2]

        print(xerror, yerror, disterror)

        direction_dictionaries['x_error'].append(xerror)
        direction_dictionaries['y_error'].append(yerror)
        direction_dictionaries['z_error'].append(disterror)

        left_right = -1 * pX * xerror
        for_back = pZ * disterror
        up_down = pY * yerror

        # To force it to be between -100 and 100
        left_right = int(np.clip(left_right,-100,100))
        for_back = int(np.clip(for_back,-100,100))
        up_down = int(np.clip(up_down,-100,100))


        direction_dictionaries['fb'].append(for_back)
        direction_dictionaries['lh'].append(left_right)
        direction_dictionaries['ud'].append(up_down)

        if disterror < 80: 
            break

        if drone.send_rc_control:
            drone.send_rc_control(left_right, for_back, up_down, 0)
        
        print(time.time() - x)
        update_rate = time.time() - x

        ####################################################################################################

        aruco.drawDetectedMarkers(frame, corners)
        aruco.drawAxis(frame, camera_matrix, distortion, rvec, tvec, 10)
        str_position = "MARKER Position x=%4.0f y=%4.0f z=%4.0f"%(tvec[0], tvec[1], tvec[2])
        cv2.putText(frame, str_position, (0,100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
   

    cv2.imshow('Frame', frame)

    # Emergency shutdown: waits for 1 milisecond and can break with q
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break


time = list(range(1, len(direction_dictionaries['x']) + 1))
time_data = [element * 0.001 for element in time]

# # Plotting
plt.figure()
plt.plot(time_data, [abs(ele) for ele in direction_dictionaries['x']])
plt.xlabel('Time (seconds)') 
plt.ylabel('Position in the x-axis (cm)') 
plt.title('Graph of position of the camera from the marker in the x-axis') 
plt.show()

plt.figure()
plt.plot(time_data, [abs(ele) for ele in direction_dictionaries['y']])
plt.xlabel('Time (seconds)') 
plt.ylabel('Position in the y-axis (cm)') 
plt.title('Graph of position of the camera from the marker in the y-axis') 
plt.show()

plt.figure
plt.plot(time_data, direction_dictionaries['z'])
plt.xlabel('Time (seconds)') 
plt.ylabel('Position in the z-axis (mm)') 
plt.title('Graph of position of the camera from the marker in the z-axis') 
plt.show()

plt.figure
plt.plot(time_data, direction_dictionaries['x_error'])
plt.xlabel('Time (seconds)') 
plt.ylabel('Error in the x-axis (cm)') 
plt.title('Graph of error of the camera from the marker in the x-axis') 
plt.show()

plt.figure
plt.plot(time_data, direction_dictionaries['y_error'])
plt.xlabel('Time (seconds)') 
plt.ylabel('Error in the y-axis (cm)') 
plt.title('Graph of error of the camera from the marker in the y-axis') 
plt.show()

plt.figure
plt.plot(time_data, direction_dictionaries['z_error'])
plt.xlabel('Time (seconds)') 
plt.ylabel('Error in the z-axis (mm)') 
plt.title('Graph of error of the camera from the marker in the z-axis') 
plt.show()

plt.figure
plt.plot(time_data, direction_dictionaries['z_error'])
plt.xlabel('Time (seconds)') 
plt.ylabel('Height (cm)') 
plt.title('Graph of height of the drone') 
plt.show()

plt.figure
plt.plot(time_data, direction_dictionaries['vx'])
plt.xlabel('Time (seconds)') 
plt.ylabel('Velocity (cm/s)') 
plt.title('Graph of velocity of the drone in the x-axis') 
plt.show()

plt.figure
plt.plot(time_data, direction_dictionaries['vy'])
plt.xlabel('Time (seconds)') 
plt.ylabel('Velocity (cm/s)') 
plt.title('Graph of velocity of the drone in the y-axis') 
plt.show()

plt.figure
plt.plot(time_data, direction_dictionaries['vz'])
plt.xlabel('Time (seconds)') 
plt.ylabel('Velocity (cm/s)') 
plt.title('Graph of velocity of the drone in the z-axis') 
plt.show()

plt.figure
plt.plot(time_data, direction_dictionaries['fb'])
plt.xlabel('Time (seconds)') 
plt.ylabel('Input Forward-Backward Velocity (cm/s)') 
plt.title('Graph of Forward-Backward Velocity of the drone') 
plt.show()

plt.figure
plt.plot(time_data, direction_dictionaries['lh'])
plt.xlabel('Time (seconds)') 
plt.ylabel('Input Left-Right Velocity (cm/s)') 
plt.title('Graph of Left-Right Velocity of the drone') 
plt.show()

plt.figure
plt.plot(time_data, direction_dictionaries['ud'])
plt.xlabel('Time (seconds)') 
plt.ylabel('Input Up-Down Velocity (cm/s)') 
plt.title('Graph of Up-Down Velocity of the drone') 
plt.show()


Tello.land(drone)
drone.end()