import time     
import brickpi3  
import grovepi  
import numpy as np

from enum import Enum

import movement
import sensors
import mapping

class Sensor(Enum):
    LEFT = 0
    RIGHT = 1
    FRONT = 2 

class Calibration:
    def __init__(self,BP,imu_calib,motor_l,motor_r,gyro,ultra_f,ultra_l,ultra_r):
        self.BP = BP
        self.motor_l = motor_l
        self.motor_r = motor_r
        self.imu = imu_calib
        self.gyro = gyro
        self.ultra_f = ultra_f
        self.ultra_l = ultra_l
        self.ultra_r = ultra_r
    @property
    def BP(self):
        return self.__BP
    @BP.setter
    def BP(self,BP):
        self.__BP = BP
    #continue

#functional tests
def calibrate(BP):
    sensors.gyroCalib(BP)
    sensors.frontUltraCalib(BP)
    sensors.irCalib()

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

def mazeNav(BP,imu_calib,speed,set_dists,kp = .2,ki = .065,bfr_dist = 5,exit_dist = 30,sensor = Sensor.RIGHT):
    '''set_dists = [front sensor stop dist, left sensor set pt, right senor set pt]'''
    try:
        errors = [-1,1,1]
        errors_p = [0,0,0]
        integs = [0,0,0]
        dt = .1

        while True:
            # center robot when trvl corridors
            while errors[0] <= 0 and errors[1] >= -bfr_dist and errors[2] >= -bfr_dist:
                errors = np.subtract(set_dists, sensors.getUltras(BP))
                integs = np.add(integs, (np.multiply(dt, np.divide(np.add(errors, errors_p), 2))))              
                outputs  = np.add(np.multiply(kp, errors), np.multiply(ki, integs))
                errors_p = errors

                if sensor == Sensor.RIGHT:
                    #apprch right wall --> (+) error --> add (+) error to right w (^ spd) & subtr (+) error to left w (v spd) 
                    #apprch left wall --> (-) error --> add (-) error to right w (v spd) & subtr (-) error to left w (^ spd)
                    movement.setSpeed(BP, speed - outputs[2], speed + outputs[2])  
                else:
                    #apprch right wall --> (-) error --> subtr (-) error to right w (^ spd) & add (-) error to left w (v spd) 
                    #apprch left wall --> negative error --> subtr (+) error to right w (v spd) & add (+) error to left w (^ spd)
                    movement.setSpeed(BP, speed + outputs[1], speed - outputs[1]) 

                time.sleep(dt)

            movement.setSpeed(BP,0,0)

            #check for junction cases
            cur_front = sensors.getUltras(BP)[0]
            cur_left = sensors.getUltras(BP)[1]
            cur_right = sensors.getUltras(BP)[2]

            #dead end
            if cur_front <= set_dists[0] and cur_left <= set_dists[1] + bfr_dist and cur_right <= set_dists[2] + bfr_dist:
                movement.turnPi(BP,180)
            #left option only
            elif cur_front <= set_dists[0] and cur_left >= set_dists[1] + bfr_dist and cur_right <= set_dists[2] + bfr_dist:
                movement.turnPi(BP,-90)
            #right option only
            elif cur_front <= set_dists[0] and cur_left <= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
                movement.turnPi(BP,90)
            #right and left options
            elif cur_front <= set_dists[0] and cur_left >= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
                movement.turnPi(BP, 90)
            #left and forward
            elif cur_front >= set_dists[0] and cur_left >= set_dists[1] + bfr_dist and cur_right <= set_dists[2] + bfr_dist:
                movement.turnPi(BP, 90)
            #right and forward
            elif cur_front >= set_dists[0] and cur_left <= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
                movement.turnPi(BP, 90)
            #4 way intersection
            elif cur_front >= set_dists[0] and cur_left >= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
                movement.turnPi(BP, 90)
            else:
                print("I don't know what to do. WTF ZACH!!!")

            movement.speedControl(BP,imu_calib,speed,exit_dist)
            time.sleep(.1)
         
    except Exception as error: 
        print("mazeNav:",error)
    except KeyboardInterrupt:
        movement.stop(BP)


def navPointsInSeq(BP,imu_calib,speed,points):
    try:
        for x in range(len(points) - 1):
            movement.pt_2_pt(BP, imu_calib, speed, points[x], points[x+1],1)

    except Exception as error: 
        print("navPointsInSeq:",error)
    except KeyboardInterrupt:
        movement.stop(BP)


if __name__ == '__main__':
    BP = brickpi3.BrickPi3()
    imu_calib = calibrate(BP)

    # movement.turnPi(BP,90)

    # movement.pt_2_pt(BP,imu_calib,5,(0,0),(3,4),1,movement.Hazard.CHECK_HAZARDS,[10,5,7])

    # pts = [(),(),(),()]
    # navPointsInSeq(BP,imu_calib,5,pts)

    # set_dists = [5,11.5,11.5]
    # mazeNav(BP,imu_calib,5,set_dists)