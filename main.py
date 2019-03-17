import time     
import brickpi3  
import grovepi  
import numpy as np

from enum import Enum
from math import cos, radians, log, pi

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
    sensors.leftUltraCalib(BP)
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

def mazeNav(BP,imu_calib,speed,set_dists,kp = .4,ki = .02,bfr_dist = 12,sensor = Sensor.LEFT,gyro_kp = .2,gyro_ki = .005):
    '''set_dists = [front sensor stop dist, left sensor set pt, right senor set pt]'''
    try:
        errors = [-1,1,1]
        errors_p = [0,0,0]
        integs = [0,0,0]
        dt = .2            #loop iteration ~= .15 + .05 sleep
        cur_angle = sensors.gyroVal(BP)
        act_dist = np.multiply(sensors.getUltras(BP), cos(radians(sensors.gyroVal(BP) - cur_angle)))
        
        while True:
            # sweep to parallel with wall
            dead_band_dist = .5
            delt_ang1 = parallelToWall(BP, cur_angle, dtheta=30, sweep_spd = 4, sensor = sensor, dt = .1)
            sensors.printGyroVal(BP)
            print("delt_ang1:",delt_ang1)
            cur_angle += delt_ang1
            
            '''
            delt_ang2 = parallelToWall(BP, cur_angle,dtheta=15, sweep_spd = 1.5, sensor = sensor, dt = .1)
            print("delt_ang2:",delt_ang2)
            #   cur_angle -= delt_ang1
            '''
            
            # while ultras[0] > set_dists[0] and ultras[1] < set_dists[1] + bfr_dist and ultras[2] < set_dists[2] + bfr_dist:
            while True:
                errors = np.subtract(set_dists, act_dist)

                if abs(errors[1]) <= dead_band_dist:
                    dead_band_dist = 4

                    # integral windup reset
                    if sensor == Sensor.LEFT:
                        integs[1] = 0
                    elif sensor == Sensor.RIGHT:
                        integs[2] = 0
                    
                    sensors.printGyroVal(BP)
                    turnPi(BP,cur_angle - sensors.gyroVal(BP))
                    sensors.printGyroVal(BP)

                    gyro_error = -1
                    gyro_error_p = 0
                    gyro_integ = 0
                    gyro_dt = .1

                    dps = (speed * (360/(7* pi)))

                    # while ultras[0] > set_dists[0] and ultras[1] < set_dists[1] + bfr_dist and ultras[2] < set_dists[2] + bfr_dist:
                    while abs(act_dist[1] - set_dists[1]) < dead_band_dist:
                        gyro_error = cur_angle - sensors.gyroVal(BP)                         #error = system (gyro) dev from desired state (target_deg)
                        gyro_integ = gyro_integ + (gyro_dt * (gyro_error + gyro_error_p)/2)  #integral feedback (trapez approx)
                        gyro_output = gyro_kp * (gyro_error) + gyro_ki * (gyro_integ)        #PI feedback response
                        gyro_error_p = gyro_error
                        
                        BP.set_motor_dps(BP.PORT_C, dps + gyro_output)   
                        BP.set_motor_dps(BP.PORT_B, dps - gyro_output) 
                        act_dist = np.multiply(sensors.getUltras(BP), cos(radians(sensors.gyroVal(BP) - cur_angle)))

                        sensors.printGyroVal(BP)
 
                        time.sleep(gyro_dt)
                    dead_band_dist = .5
                    
                integs = np.add(integs, (np.multiply(dt, np.divide(np.add(errors, errors_p), 2))))

                outputs  = np.add(np.multiply(kp, errors), np.multiply(ki, integs))
                errors_p = errors

                if sensor == Sensor.LEFT:
                    #apprch right wall --> (-) error --> subtr (-) error to right w (^ spd) & add (-) error to left w (v spd) 
                    #apprch left wall --> negative error --> subtr (+) error to right w (v spd) & add (+) error to left w (^ spd)
                    if(outputs[1] >= 0):
                        setSpeed(BP, speed + outputs[1], speed) 
                    else:
                        setSpeed(BP, speed, speed - outputs[1])
                elif sensor == Sensor.RIGHT:
                    #apprch right wall --> (+) error --> add (+) error to right w (^ spd) & subtr (+) error to left w (v spd) 
                    #apprch left wall --> (-) error --> add (-) error to right w (v spd) & subtr (-) error to left w (^ spd)
                    if(outputs[2] >= 0):
                        setSpeed(BP, speed, speed + outputs[2]) 
                    else:
                        setSpeed(BP, speed - outputs[2], speed)  

                act_dist = np.multiply(sensors.getUltras(BP), cos(radians(sensors.gyroVal(BP) - cur_angle)))

                time.sleep(.05)
            
            ###Next steps -- junctions
            '''
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
    mazeNav(BP,imu_calib,10,set_dists, kp=.5, ki=0.001 ,sensor=Sensor.LEFT)
