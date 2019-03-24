import time     
import brickpi3  
import grovepi  
import numpy as np

from enum import Enum
from math import cos, radians, log, pi

from movement import Sensor
from movement import Hazard
from movement import setSpeed
from movement import turnPi
from movement import speedControl
from movement import parallelToWall
from movement import parallelToWallDuo
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

def mazeNav(BP,imu_calib,speed,set_dists,kp = .4,ki = .02,bfr_dist = 25,sensor = Sensor.LEFT,gyro_kp = .2,gyro_ki = 0.0):
    '''set_dists = [front sensor stop dist, left sensor set pt, right senor set pt]'''
    try:
        errors = [-1,1,1]
        errors_p = [0,0,0]
        integs = [0,0,0]
        dt = .2            #loop iteration ~= .15 + .05 sleep
        cur_angle = sensors.gyroVal(BP)
        act_dists = np.multiply(sensors.getUltras(BP), cos(radians(sensors.gyroVal(BP) - cur_angle)))
        
        while True:
            # sweep to parallel with wall
            dead_band_dist = .5
            delt_ang1 = parallelToWall(BP, cur_angle, dtheta=30, sweep_spd = 2, sensor = sensor, dt = .05)
            sensors.printGyroVal(BP)
            print("delt_ang1:",delt_ang1)
            cur_angle += delt_ang1
            
            print("sonar PID init")
            while act_dists[0] > set_dists[0] and act_dists[1] < set_dists[1] + bfr_dist and act_dists[2] < set_dists[2] + bfr_dist:
                errors = np.subtract(set_dists, act_dists)

                if abs(errors[1]) <= dead_band_dist:
                    print("inside dead band")

                    dead_band_dist = 4

                    # integral windup reset
                    if sensor == Sensor.LEFT:
                        integs[1] = 0
                    elif sensor == Sensor.RIGHT:
                        integs[2] = 0
                    
                    turnPi(BP,cur_angle - sensors.gyroVal(BP))

                    gyro_error = -1
                    gyro_error_p = 0
                    gyro_integ = 0
                    gyro_dt = .1

                    dps = (speed * (360/(7* pi)))

                    print("gyro PID init")
                    while abs(act_dists[1] - set_dists[1]) < dead_band_dist and act_dists[0] > set_dists[0] and act_dists[2] < set_dists[2] + bfr_dist:
                        gyro_error = cur_angle - sensors.gyroVal(BP)                         #error = system (gyro) dev from desired state (target_deg)
                        gyro_integ = gyro_integ + (gyro_dt * (gyro_error + gyro_error_p)/2)  #integral feedback (trapez approx)
                        gyro_output = gyro_kp * (gyro_error) + gyro_ki * (gyro_integ)        #PI feedback response
                        gyro_error_p = gyro_error
                        
                        BP.set_motor_dps(BP.PORT_C, dps + gyro_output)   
                        BP.set_motor_dps(BP.PORT_B, dps - gyro_output) 
                        act_dists = np.multiply(sensors.getUltras(BP), cos(radians(sensors.gyroVal(BP) - cur_angle)))
 
                        time.sleep(gyro_dt)
                    dead_band_dist = .5
                    print("sonar PID init")

                else:    
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

                act_dists = np.multiply(sensors.getUltras(BP), cos(radians(sensors.gyroVal(BP) - cur_angle)))

                time.sleep(.05)
            
            setSpeed(BP,0,0)
            print("junction case reached")
            print("front: %d | left: %d | right: %d" % (act_dists[0],act_dists[1],act_dists[2]))
            turnPi(BP,cur_angle - sensors.gyroVal(BP))

            #check for junction cases
            # cur_front = sensors.getUltras(BP)[0]
            # cur_left = sensors.getUltras(BP)[1]
            # cur_right = sensors.getUltras(BP)[2]
            cur_front = act_dists[0]
            cur_left = act_dists[1]
            cur_right = act_dists[2]
           

            #dead end
            if cur_front <= set_dists[0] and cur_left <= set_dists[1] + bfr_dist and cur_right <= set_dists[2] + bfr_dist:
                turn_ang = 180
                print("dead end")
            #left option only
            elif cur_front <= set_dists[0] and cur_left >= set_dists[1] + bfr_dist and cur_right <= set_dists[2] + bfr_dist:
                turn_ang = -90
                print("left option only")
            #right option only
            elif cur_front <= set_dists[0] and cur_left <= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
                turn_ang = 90
                print("right option only")
            #right and left options
            elif cur_front <= set_dists[0] and cur_left >= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
                turn_ang = 90
                print("right and left options")
            #left and forward
            elif cur_front >= set_dists[0] and cur_left >= set_dists[1] + bfr_dist and cur_right <= set_dists[2] + bfr_dist:
                turn_ang = -90
                print("left and forward options")
            #right and forward
            elif cur_front >= set_dists[0] and cur_left <= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
                turn_ang = 90
                print("right and forward options")
            #4 way intersection
            elif cur_front >= set_dists[0] and cur_left >= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
                turn_ang = 90
                print("4 way intersection")
            else:
                turn_ang = 0
                print("Well, sheit...problems...")

            speedControl(BP,imu_calib,speed,10)
            cur_angle += turn_ang
            turnPi(BP,turn_ang)

            #drive forward until walls on each side and path ahead (normal)
            act_dists = np.multiply(sensors.getUltras(BP), cos(radians(sensors.gyroVal(BP) - cur_angle)))
            while act_dists[0] <= set_dists[0] or act_dists[1] >= set_dists[1] + bfr_dist or act_dists[2] >= set_dists[2] + bfr_dist:
                setSpeed(BP,speed,speed)
                act_dists = np.multiply(sensors.getUltras(BP), cos(radians(sensors.gyroVal(BP) - cur_angle)))
                time.sleep(.1)
            speedControl(BP,imu_calib,speed,5)

            setSpeed(BP,0,0)
           
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

def pocTasks(BP,imu_calib,task_num):
    ### POC 1: Naviage with walls ###
    if task_num == 1 or task_num == 12:
        set_dists = [25,11,11]
        mazeNav(BP,imu_calib,10,set_dists, kp=.5, ki=0.00 ,sensor=Sensor.LEFT)

    ### POC 2: Point turning ###
    elif task_num == 2:
        turnPi(BP,11)

    ### POC 3: Avoiding hazards ###
    elif task_num == 3:
        pt_2_pt(BP,imu_calib,5,(0,0),(12,12),5,Hazard.CHECK_HAZARDS,[30,18,30])

    ### POC 4: Point to point navigation ###
    elif task_num == 4:
        pts = [(0,0),(2,2),(-3,-1),(4,-2),(0,0)]
        navPointsInSeq(BP,imu_calib,5,pts,5)

    ### POC 5: Mapping hallway ###
    elif task_num == 5 or task_num == 56:
        pass

    ### POC 3/4: Point to point with hazards ###
    elif task_num == 34:
        pass

    ### POC ALL: Go through maze, mapping hazards and path; deposit cargo once out
    else:
        pass

    

if __name__ == '__main__':
    BP = brickpi3.BrickPi3()
    imu_calib = calibrate(BP)

    # sensors.gyroTest(BP)
    # sensors.ultrasTest(BP)
    # sensors.imuMagTest()
    # sensors.irTest()
    # movement.speedControl(BP,imu_calib,8,25)

    # pocTasks(BP,imu_calib,1)

