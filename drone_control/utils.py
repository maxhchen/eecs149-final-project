from djitellopy import Tello
import cv2
import numpy as np
import pygame

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
    return image

# This is a simple class that will help us print to the screen.
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint:
    def __init__(self, screen):
        self.reset()
        self.font = pygame.font.Font(None, 20)
        self.screen = screen

    def tprint(self, textString):
        textBitmap = self.font.render(textString, True, pygame.Color('green'))
        self.screen.blit(textBitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10


def send_special_command(drone, index_finger_value, middle_finger_value, ring_finger_value, pinky_finger_value):
    # TODO: Tune these values
    index_finger_threshhold = 40
    middle_finger_threshhold = 100
    ring_finger_threshhold = 40
    pinky_finger_threshhold = 100


    if index_finger_value > index_finger_threshhold:
        print("INDEX")
        drone.flip_back()
        return
    elif middle_finger_value > middle_finger_threshhold: 
        print("MIDDLE")
        drone.flip_forward()
        return
    elif ring_finger_value > ring_finger_threshhold:
        print("RING") 
        drone.flip_left()
        return 
    elif pinky_finger_value > pinky_finger_threshhold:
        print("PINKY")
        drone.flip_right()
        return
    else:
        print("NOTHING")
        return  

def find_special_command(index_finger_value, middle_finger_value, ring_finger_value, pinky_finger_value):
    # TODO: Tune these values
    index_finger_threshhold = 500
    middle_finger_threshhold = 4000
    ring_finger_threshhold = 500
    pinky_finger_threshhold = 500

    if index_finger_value > index_finger_threshhold:
        print("INDEX")
        return 1
    elif middle_finger_value > middle_finger_threshhold: 
        print("MIDDLE")
        return 2
    elif ring_finger_value > ring_finger_threshhold:
        print("RING") 
        return 3 
    elif pinky_finger_value > pinky_finger_threshhold:
        print("PINKY")
        return 4
    else:
        print("NOTHING")
        return 0

"""
Depending on the acceleration values from the accelerometer on the hand, it returns the RC command values 
for moving forward/backward and left/right 
"""
def find_rc_command(past_accel_values, accel_values, debug = False):
    # moving left-right is y, moving forward-backward is x, moving up-down is z
    # TODO: Tune this
    acc_x_threshhold = 0.14
    acc_y_threshhold = 0.22
    
    if past_accel_values is None: 
        return None
    accel_values = accel_values - past_accel_values 
    if debug: 
        print(accel_values)
    # return left right forward and backward 
    # 2 element tuple
    # {left/right, forward/backward}

    kp = 100 # TODO: Tune this
    if abs(accel_values[1]) > acc_y_threshhold:
        if accel_values[1] > 0:
            print("MOVING RIGHT")
            right = int(np.clip(-kp * accel_values[1],-100,100))
            return (right, 0)
        else: 
            print("MOVING LEFT")
            left = int(np.clip(-kp * accel_values[1],-100,100))
            return (left, 0)
    elif abs(accel_values[0]) > acc_x_threshhold:
        if accel_values[0] < 0:
            print("MOVING BACKWARDS")
            backward = int(np.clip(kp * accel_values[0],-100,100))
            return (0, backward)
        else: 
            print("MOVING FORWARDS")
            forward = int(np.clip(kp * accel_values[0],-100,100))
            return (0, forward)
    else:
        return (0, 0)
