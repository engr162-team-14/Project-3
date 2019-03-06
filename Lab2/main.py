import time     
import brickpi3  
import grovepi  
import numpy as np

import sensors
import movement

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


if __name__ == '__main__':
    BP = brickpi3.BrickPi3()
    imu_calib = calibrate(BP)
    movement.pt_2_pt(BP,imu_calib,5,(0,0),(4,3))

    
