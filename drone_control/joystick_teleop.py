from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time

from pygame import image

from utils import *
from detect_hands import HandDetector
from controller import Controller

# Define some colors.
FPS = 120
JOYSTICK_ID = 0
DEADZONE = .1

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

class FPVWindow:

    def __init__(self):
        pygame.init()

        # Creat pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        self.tello = init_drone()
        self.tello.send_rc_control(0, 0, 0, 0)

        self.controller = Controller(self.tello)

        # Drone velocities between -100,100
        self.forw_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0

        self.send_rc_control = False

        # Initialize the joysticks.
        pygame.joystick.init()
        self.joy = pygame.joystick.Joystick(JOYSTICK_ID)
        self.joy.init()

        # create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)
        self.clock = pygame.time.Clock()

        # Get ready to print.
        self.textPrint = TextPrint(self.screen)

        self.tracking_mode = False


    def run(self):
        frame_read = self.tello.get_frame_read()
        done = False

        while not done:
            # Possible joystick actions: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
            # JOYBUTTONUP, JOYHATMOTION
            for event in pygame.event.get(): # User did something. Necesary for joystick functions to work
                if event.type == pygame.USEREVENT + 1:
                    self.update()
                if event.type == pygame.QUIT: # If user clicked close.
                    done = True # Flag that we are done so we exit this loop.
            
            # Joystick control
            self.up_down_velocity = -self.joy.get_axis(1) * 100
            self.yaw_velocity = self.joy.get_axis(0) * 100
            self.left_right_velocity = self.joy.get_axis(2) * 100
            self.forw_back_velocity = -self.joy.get_axis(5) * 100

            left_trig = self.joy.get_axis(3)
            right_trig = self.joy.get_axis(4)

            button_a = self.joy.get_button(0)
            if button_a:
                if self.send_rc_control:
                    self.tello.land()
                else:
                    self.tello.takeoff()
                self.send_rc_control = not self.send_rc_control

            button_b = self.joy.get_button(1)
            if button_b:
                self.controller.stop()
                break

            button_x = self.joy.get_button(3)
            button_y = self.joy.get_button(3)

            dpad = self.joy.get_hat(0)
            if dpad != (0, 0) and self.send_rc_control:
                self.send_rc_control = False
                if dpad == (1, 0):
                    self.tello.flip_right()
                elif dpad == (-1, 0):
                    self.tello.flip_left()
                elif dpad == (0, 1):
                    self.tello.flip_forward()
                elif dpad == (0, -1):
                    self.tello.flip_back()
                self.send_rc_control = True

            # Displaying video + battery percentage
            frame = frame_read.frame
            frame = detector.findHands(frame)
            lmlist = detector.findPosition(frame)
            hand_center = detector.findCenter(frame, lmlist)

            text = "Battery: {}%".format(self.tello.get_battery())
            cv2.putText(frame, text, (5, 720 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = np.rot90(frame)
            frame = np.flipud(frame)
            
            self.screen.fill([0, 0, 0])
            surface = pygame.surfarray.make_surface(frame)
            self.screen.blit(surface, (0, 0))

            # Joystick Info
            for i in range(self.joy.get_numaxes()):
                axis = self.joy.get_axis(i)
                self.textPrint.tprint("Axis {} value: {:>6.3f}".format(i, axis))

            for i in range(self.joy.get_numbuttons()):
                button = self.joy.get_button(i)
                self.textPrint.tprint("Button {:>2} value: {}".format(i, button))

            # Hat position. All or nothing for direction, not a float like
            # get_axis(). Position is a tuple of int values (x, y).
            for i in range(self.joy.get_numhats()):
                hat = self.joy.get_hat(i)
                self.textPrint.tprint("Hat {} value: {}".format(i, str(hat)))

            self.textPrint.tprint(f"{(self.tello.query_attitude(), [self.tello.get_speed_x(), self.tello.get_speed_y(), self.tello.get_speed_z()], [self.tello.get_acceleration_x(), self.tello.get_acceleration_y(), self.tello.get_acceleration_z()], self.tello.get_distance_tof(), self.tello.get_height())}")

            # Hand Info
            self.textPrint.tprint(f"Hand Center: {hand_center}")

            if button_y:
                self.tracking_mode = not self.tracking_mode
                self.send_rc_control = not self.tracking_mode
                if self.tracking_mode:
                    self.controller.yaw_pid.setpoint = self.controller.drone.get_yaw()
                    self.controller.height_pid.setpoint = self.controller.drone.get_distance_tof()
                    self.controller.start()
                else:
                    self.controller.stop()

            if self.tracking_mode:
                print(f"Center: {hand_center}, Image: {frame.shape}")
                self.textPrint.tprint(f"Yaw Setpoint: {self.controller.yaw_pid.setpoint}, Height Setpoint: {self.controller.height_pid.setpoint}")
                if hand_center[0] is not None and hand_center[1] is not None:
                    image_center = np.array(frame.shape[:2]) / 2
                    x_diff = hand_center[0] - image_center[0]
                    y_diff = image_center[1] - hand_center[1]
                    yaw_p = .05
                    height_p = .1
                    self.controller.yaw_pid.setpoint = (self.controller.drone.get_yaw() + x_diff * yaw_p)
                    self.controller.height_pid.setpoint = self.controller.drone.get_distance_tof() + y_diff * height_p
                    self.textPrint.tprint(f"Xdiff: {x_diff}, Ydiff: {y_diff}")

            pygame.display.update()
            self.textPrint.reset()
            self.clock.tick(FPS)

        self.tello.land()
        self.tello.end()
        pygame.quit()

    def update(self):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            self.tello.send_rc_control(int(self.left_right_velocity), int(self.forw_back_velocity),
                int(self.up_down_velocity), int(self.yaw_velocity))


if __name__ == '__main__':
    window = FPVWindow()
    detector = HandDetector(static_mode=False, complexity=0)
    try:
        window.run()
    except Exception as e:
        print(e)
        pygame.quit()
        raise e
    finally:
        window.tello.land()
        window.tello.end()

