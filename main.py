import time     
import brickpi3  
import grovepi  
import numpy as np

from enum import Enum
from math import cos, radians

from movement import Sensor
from movement import setSpeed
from movement import turnPi
from movement import speedControl
from movement import parallelToWall
from movement import pt_2_pt
from movement import stop
import sensors
import mapping

class Calibration:
    def __init__(self,BP,imu_calib,gyro):
        self.BP = BP
        self.imu = imu_calib
        self.gyro = gyro
    @property
    def BP(self):
        return self.__BP
    @BP.setter
    def BP(self,BP):
        self.__BP = BP
    @property
    def imu(self):
        return self.__imu
    @imu.setter
    def imu(self,imu):
        self.__imu = imu
    @property
    def gyro(self):
        return self.__gyro
    @gyro.setter
    def gyro(self,gyro):
        self.__gyro = gyro
    

#functional tests
def calibrate(BP):
    #gyro = sensors.gyroCalib(BP)
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
    # BP = brickpi3.BrickPi3()
    # Cal = Calibration(BP,imu_calib,gyro)
    # return Cal

    return imu_calib

def mazeNav(BP,imu_calib,speed,set_dists,kp = .4,ki = .02,bfr_dist = 12,exit_dist = 30,sensor = Sensor.RIGHT):
    '''set_dists = [front sensor stop dist, left sensor set pt, right senor set pt]'''
    try:
        errors = [-1,1,1]
        errors_p = [0,0,0]
        integs = [0,0,0]
        dt = .15
        cur_angle = sensors.gyroVal(BP)
        act_dist = np.multiply(sensors.getUltras(BP), cos(radians(sensors.gyroVal(BP) - cur_angle)))
        
        while True:
            # center robot when trvl corridors
            turn_ang = parallelToWall(BP,cur_angle,sensor)
            print("turn_ang:",turn_ang)
            cur_angle += turn_ang + 10

            turnPi(BP, turn_ang + 10)

            # while ultras[0] > set_dists[0] and ultras[1] < set_dists[1] + bfr_dist and ultras[2] < set_dists[2] + bfr_dist:
            while True:
                errors = np.subtract(set_dists, act_dist)
                integs = np.add(integs, (np.multiply(dt, np.divide(np.add(errors, errors_p), 2))))
                outputs  = np.add(np.multiply(kp, errors), np.multiply(ki, integs))
                errors_p = errors

                if sensor == Sensor.RIGHT:
                    #apprch right wall --> (+) error --> add (+) error to right w (^ spd) & subtr (+) error to left w (v spd) 
                    #apprch left wall --> (-) error --> add (-) error to right w (v spd) & subtr (-) error to left w (^ spd)
                    setSpeed(BP, speed - outputs[2], speed + outputs[2])  
                elif sensor == Sensor.LEFT:
                    #apprch right wall --> (-) error --> subtr (-) error to right w (^ spd) & add (-) error to left w (v spd) 
                    #apprch left wall --> negative error --> subtr (+) error to right w (v spd) & add (+) error to left w (^ spd)
                    setSpeed(BP, speed + outputs[1], speed - outputs[1]) 

                act_dist = np.multiply(sensors.getUltras(BP), cos(radians(sensors.gyroVal(BP) - cur_angle)))

                time.sleep(.01)
            
            ###Next steps -- junctions
            ''''
            setSpeed(BP,0,0)
            
            #check for junction cases
            cur_front = sensors.getUltras(BP)[0]
            cur_left = sensors.getUltras(BP)[1]
            cur_right = sensors.getUltras(BP)[2]

            #dead end
            if cur_front <= set_dists[0] and cur_left <= set_dists[1] + bfr_dist and cur_right <= set_dists[2] + bfr_dist:
                turn_ang = 180
            #left option only
            elif cur_front <= set_dists[0] and cur_left >= set_dists[1] + bfr_dist and cur_right <= set_dists[2] + bfr_dist:
                turn_ang = -90
            #right option only
            elif cur_front <= set_dists[0] and cur_left <= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
                turn_ang = 90
            #right and left options
            elif cur_front <= set_dists[0] and cur_left >= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
                turn_ang = 90
            #left and forward
            elif cur_front >= set_dists[0] and cur_left >= set_dists[1] + bfr_dist and cur_right <= set_dists[2] + bfr_dist:
                turn_ang = 90
            #right and forward
            elif cur_front >= set_dists[0] and cur_left <= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
                turn_ang = 90
            #4 way intersection
            elif cur_front >= set_dists[0] and cur_left >= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
                turn_ang = 90
            else:
                print("Well, sheit.....\n")
                print("front: %d | left: %d | right: %d",cur_front,cur_left,cur_right)
            
            cur_angle += turn_ang
            turnPi(BP,turn_ang)

            #drive forward until walls on each side and path ahead (normal)
            while cur_front <= set_dists[0] or cur_left >= set_dists[1] + bfr_dist or cur_right >= set_dists[2] + bfr_dist:
                setSpeed(BP,speed,speed)
                time.sleep(.1)

            setSpeed(BP,0,0)
            '''
           
    except Exception as error: 
        print("mazeNav:",error)
    except KeyboardInterrupt:
        stop(BP)


def navPointsInSeq(BP,imu_calib,speed,points,length_conv = 5):
    try:
        for x in range(len(points) - 1):
            pt_2_pt(BP, imu_calib, speed, points[x], points[x+1],length_conv)

    except Exception as error: 
        print("navPointsInSeq:",error)
    except KeyboardInterrupt:
        stop(BP)


if __name__ == '__main__':
    BP = brickpi3.BrickPi3()
    imu_calib = calibrate(BP)

    # sensors.gyroTest(BP)
    # sensors.ultrasTest(BP)
    # sensors.imuMagTest()
    # sensors.irTest()
    # movement.speedControl(BP,imu_calib,8,25)

    # turnPi(BP,11)

    # pts = [(0,0),(2,2),(-3,-1),(4,-2),(0,0)]
    # navPointsInSeq(BP,imu_calib,5,pts,5)

    # movement.pt_2_pt(BP,imu_calib,5,(0,0),(12,12),5,movement.Hazard.CHECK_HAZARDS,[30,18,30])

    set_dists = [10,11.5,11.5]
    mazeNav(BP,imu_calib,6,set_dists, kp=.4, ki=.01,sensor=Sensor.LEFT)
