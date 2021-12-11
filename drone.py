
from djitellopy import Tello
import cv2
import math
import numpy as np

def init_drone():
    tello = Tello()
    tello.connect()
    print("Battery level:", tello.get_battery())
    if tello.get_battery() < 5: 
        return "Low Battery: Go recharge"
    tello.streamoff()
    tello.streamon()
    return tello 

def send_rc_command(drone, accel_values, gyro_values):
    # moving up-down is y, moving left-right is x, moving forward-backward is z
    acc_x_threshhold = 0.3
    acc_y_threshhold = 0.3
    acc_z_threshhold = 0.3

    gy_x_threshhold = 10
    gy_y_threshhold = 10 
    gy_z_threshhold = 10

    kp = 0.5
    if abs(accel_values[0]) > acc_x_threshhold:
        if accel_values[0] < 0:
            left = int(np.clip(kp * accel_values[0],-100,100))
            assert(left < 0)
            drone.send_rc_control(left, 0, 0, 0)
        else: 
            right = int(np.clip(kp * accel_values[0],-100,100))
            assert(right > 0)
            drone.send_rc_control(right, 0, 0, 0)
        return 
    elif abs(accel_values[1]) > acc_y_threshhold:
        if accel_values[1] < 0:
            down = int(np.clip(kp * accel_values[1],-100,100))
            assert(down < 0)
            drone.send_rc_control(0, 0, down, 0)
        else: 
            up = int(np.clip(kp * accel_values[1],-100,100))
            assert(up > 0)
            drone.send_rc_control(0, 0, up, 0)
        return 
    elif abs(accel_values[2]) > acc_z_threshhold:
        if accel_values[2] < 0:
            backward = int(np.clip(kp * accel_values[2],-100,100))
            assert(backward < 0)
            drone.send_rc_control(0, backward, 0, 0)
        else: 
            forward = int(np.clip(kp * accel_values[2],-100,100))
            assert(forward > 0)
            drone.send_rc_control(0, forward, 0, 0)
        return
    elif abs(accel_values[2]) > acc_z_threshhold and abs(accel_values[1]) > acc_y_threshhold and abs(accel_values[0]) > acc_x_threshhold:
        if accel_values[0] < 0:
            yaw_left = int(np.clip(kp * accel_values[0],-100,100))
            assert(yaw_left < 0)
            drone.send_rc_control(0, 0, 0, yaw_left)
        else: 
            yaw_right = int(np.clip(kp * accel_values[0],-100,100))
            assert(yaw_right > 0)
            drone.send_rc_control(0, 0, 0, yaw_right)
        return
    else:
        drone.send_rc_control(0, 0, 0, 0)
