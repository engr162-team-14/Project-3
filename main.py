import time     
import brickpi3  
import grovepi  
import numpy as np

import movement
import sensors

from MPU9250 import MPU9250


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

def maze_nav_pi(BP,set_dists,speed,k):
    try:
        error = -1
        error_p = 0
        integ = 0
        dt = .1

        while True:
            errors = np.subtract(set_dists - ultra_sens(...))
                        
            integ = integ + (dt * (error + error_p)/2)                #trapezoidal approx
            deriv = (error - error_p)/dt
            output  = (kp * error) + (ki * integral)  + (kd * deriv)
            error_p = error
            
                

    except Exception as error: 
        print("maze_nav_pi:",error)
    except KeyboardInterrupt:
        stop(BP)



if __name__ == '__main__':
    BP = brickpi3.BrickPi3()
    imu_calib = calibrate(BP)