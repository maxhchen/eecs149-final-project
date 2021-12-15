import cv2
import pygame
import numpy as np
import time

from pygame import image

from utils import *
from detect_hands import HandDetector
from controller import Controller
from sensor_glove import SensorGlove

# Define some colors.
FPS = 120
JOYSTICK_ID = 0
DEADZONE = .1
class FPVWindow:

    def __init__(self):
        pygame.init()

        # Creat pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        self.glove = SensorGlove()
        print("Started Arduino processing")
        self.glove.calibrate_imu(50)
        self.glove.calibrated = True
        print("Finished Calibrating IMU")

        self.tello = init_drone()
        self.tello.send_rc_control(0, 0, 0, 0)

        self.controller = Controller(self.tello)
        self.detector = HandDetector(static_mode=False, complexity=1)

        # Drone velocities between -100,100
        self.forw_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0

        self.send_rc_control = False
        self.tracking_mode = False
        self.hand_mode = False

        # Initialize the joysticks.
        pygame.joystick.init()
        self.joy = pygame.joystick.Joystick(JOYSTICK_ID)
        self.joy.init()

        # create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)
        self.clock = pygame.time.Clock()

        # Create screen printer
        self.textPrint = TextPrint(self.screen)



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
                self.detector.stop()
                break

            button_x = self.joy.get_button(2)
            button_y = self.joy.get_button(3)

            if button_y:
                self.tracking_mode = not self.tracking_mode
                self.hand_mode = False
                self.send_rc_control = not self.tracking_mode
                if self.tracking_mode:
                    self.detector.start()
                    self.controller.yaw_pid.setpoint = self.controller.drone.get_yaw()
                    self.controller.height_pid.setpoint = self.controller.drone.get_distance_tof()
                    self.controller.start()
                else:
                    self.detector.stop()
                    self.controller.stop()
            elif button_x:
                self.hand_mode = not self.hand_mode
                if self.tracking_mode:
                    self.controller.stop()
                    self.detector.stop()
                self.tracking_mode = False

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
            self.detector.image = frame

            if self.tracking_mode and self.detector.hand_image is not None:
                frame = self.detector.hand_image

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = np.rot90(frame)
            frame = np.flipud(frame)
            surface = pygame.surfarray.make_surface(frame)
            self.screen.fill([0, 0, 0])
            self.screen.blit(surface, (0, 0))

            self.textPrint.tprint("Battery: {}%".format(self.tello.get_battery()))

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
            self.textPrint.tprint(f"Hand Center: {self.detector.hand_center}")

            if self.tracking_mode:
                hand_center = self.detector.hand_center
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

            if self.hand_mode:
                self.glove.get_data()
                if self.glove.calibrated:
                    self.yaw_velocity = 0
                    self.up_down_velocity = 0
                    self.left_right_velocity = self.glove.left_right
                    self.forw_back_velocity = self.glove.forward_backward
                if self.glove.finger != 0:
                    old_rc_control = self.send_rc_control
                    self.send_rc_control = False
                    try:
                        if self.glove.finger == SensorGlove.INDEX: 
                            self.tello.flip_right()
                        elif self.glove.finger == SensorGlove.MIDDLE:
                            self.tello.flip_left()
                        elif self.glove.finger == SensorGlove.RING: 
                            self.tello.flip_forward()
                        elif self.glove.finger == SensorGlove.PINKY: 
                            self.tello.flip_back()
                    except Exception as e:
                        print(e)
                    self.send_rc_control = old_rc_control

            self.textPrint.tprint(f"LR: {self.left_right_velocity}, FB: {self.forw_back_velocity}, UD: {self.up_down_velocity}, Yaw: {self.yaw_velocity}")
            self.textPrint.tprint(f"Tracking: {self.tracking_mode}, Hand: {self.hand_mode}")

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
    try:
        window.run()
    finally:
        pygame.quit()
        window.controller.stop()
        window.send_rc_control = False
        window.tello.land()
        window.tello.end()
