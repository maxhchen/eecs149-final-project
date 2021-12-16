import time
import threading
import logging

import numpy as np

from utils import *

tello_logger = logging.getLogger('djitellopy')
tello_logger.setLevel(logging.WARN)
class PID:
    """Class that handles PID looping and outputs. Handles both scalar and arrays"""

    def __init__(self, p, i , d, sat, goal_thresh=1.0, setpoint=0.0, state_getter=None, wrap=False):
        self.p = np.array(p)
        self.i = np.array(i)
        self.d = np.array(d)
        self.sat = sat
        self.setpoint = setpoint
        self.goal_thresh = goal_thresh
        self.cur_val = state_getter
        self.wrap = wrap
        self.reset()

    def reset(self):
        self.prevError = 0.0
        self.prevIntegral = 0.0
        self.prevTime = 0.0

    def calculate(self, curTime):
        error = self.setpoint - self.cur_val()
        if self.wrap:
            while error > 180:
                error -= 360
            while error < -180:
                error += 360
        p_term = self.p*error
        if curTime - self.prevTime != 0:
            d_term = self.d*(error - self.prevError)/(curTime - self.prevTime)
        else:
            d_term = 0

        integral = self.prevIntegral + error*(curTime - self.prevTime)

        self.prevError = error
        self.prevTime = curTime
        self.prevIntegral = integral

        u = p_term + self.i*integral + d_term
        if np.linalg.norm(u) > self.sat:
            # Reached saturation point. Reset integrator windup
            u = u / np.linalg.norm(u) * self.sat
            self.prevIntegral = 0.0
        
        # print("setpoint cur_val u error:", self.setpoint, self.cur_val(), u, error, flush=True)
        return u

    def on_target(self):
        diff = np.abs(self.setpoint - self.cur_val())
        if self.wrap:
            while diff > 180:
                diff -= 360
            while diff < -180:
                diff += 360
            diff = np.abs(diff)
        return diff <= self.goal_thresh

class Controller:

    def __init__(self, drone=None):
        if drone is None:
            drone = init_drone()
            drone.streamoff()
            drone.takeoff()
        self.drone = drone

        self.init_yaw = drone.get_yaw()
        self.init_height = drone.get_height()

        self.yaw_pid = PID(p=2.5, i=0, d=0, sat=float('inf'), goal_thresh=2, setpoint=self.init_yaw, state_getter=drone.get_yaw, wrap=True)
        self.height_pid = PID(p=2.0, i=0, d=0, sat=float('inf'), goal_thresh=2, setpoint=self.init_height, state_getter=drone.get_distance_tof, wrap=False)

        self.x_pid = PID(p=-.5, i=0.00, d=0, sat=100, goal_thresh=2, setpoint=0, state_getter=drone.get_acceleration_x)
        self.y_pid = PID(p=-.5, i=0.00, d=0, sat=100, goal_thresh=2, setpoint=0, state_getter=drone.get_acceleration_y)

        self.start_time = time.time()
        
        self.stop_event = threading.Event()

        self.yaw_vel = 0
        self.z_vel = 0

    def start(self, send=False):
        self.yaw_pid.reset()
        self.height_pid.reset()
        self.x_pid.reset()
        self.y_pid.reset()
        self.stop_event.clear()
        self.start_time = time.time()
        self.send = send
        self.thread = threading.Thread(target=self.send_control, args=())
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.drone.send_rc_control(0, 0, 0, 0)

    def send_control(self):
        # Estimate bias of acceleromters and remove it from sensor reading for PID
        x_accel = []
        y_accel = []
        for i in range(50):
            x_accel.append(self.drone.get_acceleration_x())
            y_accel.append(self.drone.get_acceleration_y())
            time.sleep(0.01)
        x_bias = sum(x_accel) / len(x_accel)
        y_bias = sum(y_accel) / len(y_accel)
        x_corrected = lambda: self.drone.get_acceleration_x() - x_bias
        y_corrected = lambda: self.drone.get_acceleration_y() - y_bias
        self.x_pid.cur_val = x_corrected
        self.y_pid.cur_val = y_corrected
        
        while not self.stop_event.is_set():
            self.yaw_vel = self.yaw_pid.calculate(time.time() - self.start_time)
            self.z_vel = self.height_pid.calculate(time.time() - self.start_time)
            x_vel = self.x_pid.calculate(time.time() - self.start_time)
            y_vel = self.y_pid.calculate(time.time() - self.start_time)
            # print("RC Vel:", round(x_vel, 2), round(y_vel, 2), round(x_corrected(), 2), round(y_corrected(), 2))
            if self.send:
                self.drone.send_rc_control(0, 0, int(self.z_vel), int(self.yaw_vel))
            time.sleep(0.01)
        print("Finished autonomous control")

    def on_target(self):
        return self.yaw_pid.on_target() and self.height_pid.on_target() #and self.x_pid.on_target() and self.y_pid.on_target()

if __name__ == '__main__':
    controller = Controller()
    drone = controller.drone
    
    controller.yaw_pid.setpoint = 90 + controller.init_yaw
    controller.height_pid.setpoint = controller.init_height

    try:
        controller.start(send=True)
        # controller.height_pid.setpoint = 100
        while not controller.on_target():
            # if controller.x_pid.on_target():
            #     print("x setpoint achieved")
            # if controller.y_pid.on_target():
            #     print("y setpoint achieved")
            print("Battery:", drone.get_battery(), "Init val:", controller.init_height, controller.init_yaw, "Cur val:", drone.get_yaw(), drone.get_distance_tof(), drone.get_height())
            # print([drone.get_speed_x(), drone.get_speed_y(), drone.get_speed_z()], [drone.get_acceleration_x(), drone.get_acceleration_y(), drone.get_acceleration_z()])
            time.sleep(.05)
        print("Controller on target")
        controller.stop()
    except KeyboardInterrupt:
        controller.stop()

    drone.end()
