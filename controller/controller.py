import time

import numpy as np

from utils import *


class PID:
    """Class that handles PID looping and outputs. Handles both scalar and arrays"""

    def __init__(self, p, i , d, sat, goal_thresh=1.0, setpoint=0.0, state_getter=None):
        self.p = np.array(p)
        self.i = np.array(i)
        self.d = np.array(d)
        self.sat = sat
        self.setpoint = setpoint
        self.goal_thresh = goal_thresh
        self.curVal = state_getter
        self.reset()

    def reset(self):
        self.prevError = 0.0
        self.prevIntegral = 0.0
        self.prevTime = 0.0

    def setSetpoint(self, setpoint):
        self.setpoint = setpoint

    def calculate(self, curTime):
        error = self.setpoint - self.curVal()
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
        
        print("setpoint curVal u error:", self.setpoint, self.curVal(), u, error)
        return u

    def onTarget(self):
        return np.abs(self.setpoint - self.curVal()) < self.goal_thresh

if __name__ == '__main__':
    drone = init_drone()
    drone.streamoff()
    drone.takeoff()

    vel_setpoint = np.zeros(3)
    cur_vel = np.array([drone.get_speed_x(), drone.get_speed_y(), drone.get_speed_z()])
    init_yaw = drone.get_yaw()
    init_height = drone.get_distance_tof()
    print("init height:", init_height)
    rotate_deg = 90

    yaw_pid = PID(p=2.5, i=0, d=0, sat=float('inf'), goal_thresh=2, setpoint=init_yaw + rotate_deg, state_getter=drone.get_yaw)
    height_pid = PID(p=1.0, i=0, d=0, sat=float('inf'), goal_thresh=2, setpoint=150, state_getter=drone.get_distance_tof)
    # z_pid = PID(p=1.0, i=0, d=0, sat=float('inf'), goal_thresh=1, setpoint=0, state_getter=drone.get_speed_z)

    start_time = time.time()
    yaw_pid.reset()
    height_pid.reset()
    try:
        while not (height_pid.onTarget() and yaw_pid.onTarget()):
            yaw_vel = yaw_pid.calculate(time.time() - start_time)
            z_vel = height_pid.calculate(time.time() - start_time)
            print(drone.get_yaw(), [drone.get_speed_x(), drone.get_speed_y(), drone.get_speed_z()], [drone.get_acceleration_x(), drone.get_acceleration_y(), drone.get_acceleration_z()], drone.get_distance_tof(), drone.get_height())
            drone.x _control(0, 0, int(z_vel), int(yaw_vel))
            if yaw_pid.onTarget():
                print("yaw setpoint achieved")
            if height_pid.onTarget():
                print("height setpoint achieved")
            time.sleep(.01)
    except KeyboardInterrupt:
        pass
    

    drone.land()
    drone.end()
