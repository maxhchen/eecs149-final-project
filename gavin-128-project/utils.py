from djitellopy import Tello
import cv2
import numpy as np

def init_drone():
    tello = Tello()
    tello.connect()
    tello.for_back_velocity = 0
    tello.left_right_velocity = 0
    tello.up_down_velocity = 0 
    tello.yaw_velocity = 0
    tello.speed = 0
    if tello.get_battery() < 5: 
        print(tello.get_battery())
        return "Low Battery: Go recharge"
    tello.streamoff()
    tello.streamon()
    return tello 

def get_tello_frame(drone, width = 360, height = 240): # width and height for image resizing 
    tello_frame = drone.get_frame_read()
    image = tello_frame.frame 
    processing_img = cv2.resize(image, (width, height))
    return processing_img

def trackBalloon(drone, pid, info, error, intError, prevError):
    # PID constants
    pidX = pid[0]
    pidY = pid[1]
    pidZ = pid[2]

    # PID Controller (Velocity Controller)
    velX = pidX[0]*error[0] + pidX[1]*intError[0] + pidX[2]*(error[0] - prevError[0])
    velY = pidY[0]*error[1] + pidY[1]*intError[1] + pidY[2]*(error[1] - prevError[1])
    velZ = pidZ[0]*error[2] + pidZ[1]*intError[2] + pidZ[2]*(error[2] - prevError[2])

    # Calibration and saturation (Velocity limited to +- 100)
    maxVel = 100
    minVel = -100

    velX = -1 * velX
    velY = -1 * velY
    velZ = -1 * velZ

    if velX > maxVel: 
        velX = maxVel   
    if velY > maxVel: 
        velY = maxVel 
    if velZ > maxVel: 
        velZ = maxVel 

    if velX < minVel: 
        velX = minVel   
    if velY < minVel: 
        velY = minVel 
    if velZ < minVel: 
        velZ = minVel

    print(error)
    print(int(velX))
    print(int(velY))
    # Figure out what the first statement means
    if info[0] != 0:
        drone.left_right_velocity = int(velX)
        drone.for_back_velocity = 0
        drone.up_down_velocity = int(velY)
        drone.yaw_velocity = 0
    else:
        # If error is not detected, drone should stay in place mid-flight
        drone.left_right_velocity = 0
        drone.for_back_velocity = 0
        drone.up_down_velocity = 0
        drone.yaw_velocity = 0
        error = [0.0, 0.0, 0.0]
        

    # Send assigned values to the drone object.
    if drone.send_rc_control:
        drone.send_rc_control(drone.left_right_velocity,
                              drone.for_back_velocity,
                              drone.up_down_velocity,
                              drone.yaw_velocity)

    return error
