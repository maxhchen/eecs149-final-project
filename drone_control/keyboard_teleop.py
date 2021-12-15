from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
import threading

from utils import *
from controller import Controller
from detect_hands import HandDetector
from sensor_glove import SensorGlove

# Speed of the drone
S = 60
# Frames per second of the pygame window display
# A low number also results in input lag, as input information is processed once per frame.
FPS = 120


class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations (yaw)
            - W and S: Up and down.
    """

    def __init__(self):
        # Init pygame
        pygame.init()

        # Creat pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        self.glove = SensorGlove()
        print("Started Arduino processing")
        self.glove.calibrate_imu(50)
        self.glove.calibrated = True
        print("Finished Calibrating IMU")

        # Init Tello object that interacts with the Tello drone
        self.tello = init_drone()
        self.tello.send_rc_control(0, 0, 0, 0)

        self.controller = Controller(self.tello)
        self.tracking_mode = False
        self.hand_mode = False
        self.textPrint = TextPrint(self.screen)
        self.detector = HandDetector(static_mode=False, complexity=1)

        # Drone velocities between -100~100
        self.forw_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0

        self.send_rc_control = False

        # create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)

    def run(self):
        frame_read = self.tello.get_frame_read()

        should_stop = False
        while not should_stop:

            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:
                    self.update()
                elif event.type == pygame.QUIT:
                    should_stop = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        should_stop = True
                    else:
                        self.keydown(event.key)
                elif event.type == pygame.KEYUP:
                    self.keyup(event.key)

            if frame_read.stopped:
                break

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
            time.sleep(1 / FPS) # TODO: Maybe make busy wait

        # Call it always before finishing. To deallocate resources.
        self.tello.end()

    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP:  # set forward velocity
            self.forw_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.forw_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw counter clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw clockwise velocity
            self.yaw_velocity = S

    def keyup(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.forw_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # takeoff
            if not self.send_rc_control:
                self.tello.takeoff()
                self.send_rc_control = True
        elif key == pygame.K_l:  # land
            self.send_rc_control = False
            self.controller.stop()
            self.detector.stop()
            self.hand_mode = False
            self.tracking_mode = False
            self.tello.land()
        elif key == pygame.K_g:
            self.send_rc_control = False
            self.tello.flip_forward()
            self.send_rc_control = True
        elif key == pygame.K_b:
            self.send_rc_control = False
            self.tello.flip_back()
            self.send_rc_control = True
        elif key == pygame.K_v:
            self.send_rc_control = False
            self.tello.flip_left()
            self.send_rc_control = True
        elif key == pygame.K_n:
            self.send_rc_control = False
            self.tello.flip_right()
            self.send_rc_control = True
        elif key == pygame.K_y:
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
        elif key == pygame.K_h:
            self.hand_mode = not self.hand_mode
            if self.tracking_mode:
                self.controller.stop()
                self.detector.stop()
            self.tracking_mode = False

    def update(self):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.forw_back_velocity,
                self.up_down_velocity, self.yaw_velocity)


if __name__ == '__main__':
    frontend = FrontEnd()
    try:
        frontend.run()
    finally:
        pygame.quit()
        frontend.controller.stop()
        frontend.send_rc_control = False
        frontend.tello.land()
        frontend.tello.end()

