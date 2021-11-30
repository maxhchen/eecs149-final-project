from djitellopy import Tello, tello
import cv2
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

def get_tello_frame(drone, width = 360, height = 240): # width and height for image resizing 
    tello_frame = drone.get_frame_read()
    image = tello_frame.frame 
    image = cv2.resize(image, (width, height))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # image = np.rot90(image)
    # image = np.flipud(image)
    return image
