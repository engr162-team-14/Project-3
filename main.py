import time     
import brickpi3  
import grovepi  
import numpy as np

import movement
import sensors

from enum import Enum, auto


class Sensor(Enum):
    LEFT = auto()
    RIGHT = auto()
    FRONT = auto()

class State(Enum):
    WALL = auto()
    EXPL = auto()
    UNEXPL = auto()

class Surrounding:
    def __init__(self, up = State.UNEXPL, down = State.EXPL, left = State.WALL, right = State.WALL):
        self.up = up
        self.down = down
        self.left = left
        self.right = right
    def get_up(self):
        return self.up
    def get_down(self):
        return self.down
    def get_left(self):
        return self.left
    def get_right(self):
        return self.right    

#functional tests
def calibrate(BP):
    sensors.gyroCalib(BP)

    calib = sensors.imuCalib()
    imu_calib = {
        "mpu": calib[0],
        "accel": calib[1],
        "gyro": calib[2],
        "mag": calib[3],
        "flter": calib[4],
        "biases": calib[5],
        "dly": calib[6],
        "std": calib[7]
    }
    return imu_calib

def maze_nav_pi(BP,speed,set_dists,kp,ki,kd,sensor = Sensor.RIGHT):
    #set_dists = [front sensor stop dist, left sensor set pt, right senor set pt]
    try:
        errors = [-1,1,1]
        errors_p = [0,0,0]
        integs = [0,0,0]
        surr = Surrounding()
        dt = .1

        #may need to clean this up
        while True:
            #                     need to define   v      and          v
            # center robot when trvl corridors
            while errors[0] <= 0 and errors[1] >= -5 and errors[2] >= -5:
                errors = np.subtract(set_dists, **ultra_sens(....)**)
                integ = np.add(integ, (np.multiply(dt, np.divide(np.add(error, error_p), 2))))              
                #deriv = np.divide(np.subtract(error, error_p), dt)
                output  = np.add(np.multiply(kp, error), np.multiply(ki, integ))
                errors_p = errors

                if sens == Sensor.RIGHT:
                    #apprch right wall --> (+) error --> add (+) error to right w (^ spd) & subtr (+) error to left w (v spd) 
                    #apprch left wall --> (-) error --> add (-) error to right w (v spd) & subtr (-) error to left w (^ spd)
                    movement.setSpeed(BP, speed - output[2], speed + output[2])  
                else:
                    #apprch right wall --> (-) error --> subtr (-) error to right w (^ spd) & add (-) error to left w (v spd) 
                    #apprch left wall --> negative error --> subtr (+) error to right w (v spd) & add (+) error to left w (^ spd)
                    movement.setSpeed(BP, speed + output[1], speed - output[1]) 
            
            #check for junction cases
            
            
    except Exception as error: 
        print("maze_nav_pi:",error)
    except KeyboardInterrupt:
        movement.stop(BP)



if __name__ == '__main__':
    BP = brickpi3.BrickPi3()
    imu_calib = calibrate(BP)