import time     
import brickpi3  
import grovepi  
import numpy as np

from enum import Enum
from math import cos, radians, log, pi

from movement import Hazard
from movement import setSpeed
from movement import turnPi
from movement import speedControl
from movement import parallelToWall
from movement import parallelToWallDuo
from movement import pt_2_pt
from movement import cargoRelease
from movement import stop
from movement import Sensor
from movement import hazardCheck

from sensors import gyroCalib
from sensors import gyroVal
from sensors import gyroTest
from sensors import printGyroVal
from sensors import leftUltraCalib
from sensors import leftUltraTest
from sensors import getUltras
from sensors import ultrasTest
from sensors import imuCalib
from sensors import imuMagFiltered
from sensors import imuMagTest
from sensors import irCalib
from sensors import irTest
from sensors import irVal

from mapping import Map
from mapping import Dir
from mapping import State
    
def calibrate(BP):
    '''
    Description: Calibrates gyroscope, ultrasonic, IR, and imu sensors \n 
    Return value: hazardCheck returns imu_calib \n
               hazard_type -- Dictionary that holds calibration
                              data for each sensor component of imu
    '''
    BP.offset_motor_encoder(BP.PORT_D,BP.get_motor_encoder(BP.PORT_D))
    BP.set_motor_position(BP.PORT_D, 0)

    gyroCalib(BP)
    leftUltraCalib(BP)
    irCalib()

    calib = imuCalib()
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

def wallGuide(map, cur_angle, act_dists, set_dists, bfr_dist, sensor, speed, kp, ki, gyro_kp, gyro_ki):
    '''
    Description: Navigates within a corridor using two PI controllers based on distance from the set point (between the walls) \n 
    Return value: wallGuide returns cur_angle, act_dists \n
               cur_angle -- integer that corresponds to current gyroscope position
               act_dists -- array of integers that correspong to front, left, and right ultrasonic sensor readings 
    '''
    try:
        dead_band_dist = .5            # distance from set point where controller switches from ultras to gyroscope
        errors = [-1,1,1]              # ultrasonic process errors
        errors_p = [0,0,0]
        integs = [0,0,0]
        dt = .2                        # loop iteration ~= .18 + .01 sleep
        hazard_type = None             # enum that signifies type of hazard identified (None if no hazard)
        has_encountered_haz = False

        print("outisde dead band --> ultrasonic PI control init")

        # offset motor encoders to track distance traveled current location tracking
        BP.offset_motor_encoder(BP.PORT_C,BP.get_motor_encoder(BP.PORT_C))
        BP.offset_motor_encoder(BP.PORT_B,BP.get_motor_encoder(BP.PORT_B))

        # travels in corridor as long as left and right sensors see walls and front sensor does not (within set distance)
        while act_dists[0] > set_dists[0] and act_dists[1] < set_dists[1] + bfr_dist and act_dists[2] < set_dists[2] + bfr_dist:
            errors = np.subtract(set_dists, act_dists)

            #if encoder value says weve traveled 40 cm --> reset initial encoder value and update current location
            if (BP.get_motor_encoder(BP.PORT_C) + BP.get_motor_encoder(BP.PORT_C)) / 2 < -(40 * (360/(7* pi))):
                BP.offset_motor_encoder(BP.PORT_C,BP.get_motor_encoder(BP.PORT_C))
                BP.offset_motor_encoder(BP.PORT_B,BP.get_motor_encoder(BP.PORT_B))
                map.updateLocation()
                print("Update loc -- ultra PI")

            # check for hazards if not already identified one in given corridor
            if hazard_type == None:
                hazard_type, hazard_val =  hazardCheck(BP, imu_calib)

            # switch to gyro PI controller if within dead band
            if abs(errors[1]) <= dead_band_dist:
                print("inside dead band -> Gyro PI control init")
                dead_band_dist = 2  # allow 2cm deviance before returning to ultrasonic PI control

                # integral windup reset and return to parallel w/ wall angle
                integs = [0,0,0]
                turnPi(BP,cur_angle - gyroVal(BP))

                gyro_error = -1
                gyro_error_p = 0
                gyro_integ = 0
                gyro_dt = .1
                # navigates corridor while maintaining angle so long as a junction/hazard is not reached or devaite more than dead_band_dist from set pt
                while (hazard_type == None or has_encountered_haz == True) and abs(act_dists[1] - set_dists[1]) < dead_band_dist and act_dists[0] > set_dists[0] and act_dists[2] < set_dists[2] + bfr_dist:
                    #if encoder value says weve traveled 40 cm --> reset initial encoder value and update current location
                    if (BP.get_motor_encoder(BP.PORT_C) + BP.get_motor_encoder(BP.PORT_C)) / 2 < -(40 * (360/(7* pi))):
                        BP.offset_motor_encoder(BP.PORT_C,BP.get_motor_encoder(BP.PORT_C))
                        BP.offset_motor_encoder(BP.PORT_B,BP.get_motor_encoder(BP.PORT_B))
                        map.updateLocation()
                        print("Update loc -- gyro PI")

                    gyro_error = cur_angle - gyroVal(BP)                                 #error = system (gyro) dev from desired state (target_deg)
                    gyro_integ = gyro_integ + (gyro_dt * (gyro_error + gyro_error_p)/2)  #integral feedback (trapez approx)
                    gyro_output = gyro_kp * (gyro_error) + gyro_ki * (gyro_integ)        #PI feedback response
                    gyro_error_p = gyro_error

                    # control signal
                    if(gyro_output >= 0):
                        setSpeed(BP, speed + gyro_output, speed) 
                    else:
                        setSpeed(BP, speed, speed - gyro_output)
                    
                    # check for hazards and get new ultra distances
                    if hazard_type == None:
                        hazard_type, hazard_val =  hazardCheck(BP, imu_calib)
                    act_dists = np.multiply(getUltras(BP), cos(radians(gyroVal(BP) - cur_angle)))

                    time.sleep(gyro_dt)
                dead_band_dist = .5   # reset deadband distance for ultrasonic controller
            else:    
                integs = np.add(integs, (np.multiply(dt, np.divide(np.add(errors, errors_p), 2))))

                outputs  = np.add(np.multiply(kp, errors), np.multiply(ki, integs))
                errors_p = errors

                # control signal
                if sensor == Sensor.LEFT:
                    # apprch right wall --> (-) error --> subtr (-) error to right w (^ spd) & add (-) error to left w (v spd) 
                    if(outputs[1] >= 0):
                        setSpeed(BP, speed + outputs[1], speed) 
                    # apprch left wall --> (+) error --> subtr (+) error to right w (v spd) & add (+) error to left w (^ spd)
                    else:
                        setSpeed(BP, speed, speed - outputs[1])
                elif sensor == Sensor.RIGHT:
                    # apprch right wall --> (+) error --> add (+) error to right w (^ spd) & subtr (+) error to left w (v spd) 
                    if(outputs[2] >= 0):
                        setSpeed(BP, speed, speed + outputs[2]) 
                    # apprch left wall --> (-) error --> add (-) error to right w (v spd) & subtr (-) error to left w (^ spd)
                    else:
                        setSpeed(BP, speed - outputs[2], speed)  
            
            # handle hazard one has not yet been encountered in this scope
            if hazard_type != None and not has_encountered_haz:
                setSpeed(BP,0,0)
                print("Hazard located")

                has_encountered_haz = True
                cur_angle += 180
                turnPi(BP,180)

                map.addHazard(hazard_type,hazard_val)
                map.cur_direc = Dir((map.cur_direc.value - 180 // 90) % 4)                # change current direction property based on turn_ang
            act_dists = np.multiply(getUltras(BP), cos(radians(gyroVal(BP) - cur_angle))) # cos(relative theta) of all ultrasonic distances

            time.sleep(.01)
        setSpeed(BP,0,0)

        return cur_angle, act_dists
    except Exception as error: 
        print("wallGuide:",error)
    except KeyboardInterrupt:
        stop(BP)
        map.pushInfo()

def handleJunction(map, set_dists, cur_angle, cur_front, cur_left, cur_right, bfr_dist):
    try:
        # Take path based on junction type
        #dead end
        if cur_front <= set_dists[0] + bfr_dist and cur_left <= set_dists[1] + bfr_dist and cur_right <= set_dists[2] + bfr_dist:
            turn_ang = 180
            speedControl(BP,imu_calib,-10,5)
            print("dead end")
        #left option only
        elif cur_front <= set_dists[0] + bfr_dist and cur_left >= set_dists[1] + bfr_dist and cur_right <= set_dists[2] + bfr_dist:
            turn_ang = -90
            print("left option only")
        #right option only
        elif cur_front <= set_dists[0] + bfr_dist and cur_left <= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
            turn_ang = 90
            print("right option only")
        #right and left options
        elif cur_front <= set_dists[0] + bfr_dist and cur_left >= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
            turn_ang = 90
            # Demo version --> turn_ang = - 90
            print("right and left options")
        #left and forward
        elif cur_front >= set_dists[0] + bfr_dist and cur_left >= set_dists[1] + bfr_dist and cur_right <= set_dists[2] + bfr_dist:
            turn_ang = 0
            print("left and forward options")
        #right and forward
        elif cur_front >= set_dists[0] + bfr_dist and cur_left <= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
            turn_ang = 90
            # Demo version --> turn_ang = 0
            print("right and forward options")
        #4 way intersection
        elif cur_front >= set_dists[0] + bfr_dist and cur_left >= set_dists[1] + bfr_dist and cur_right >= set_dists[2] + bfr_dist:
            turn_ang = 90
            # Demo version --> turn_ang = - 90
            print("4 way intersection")
        else:
            turn_ang = 0
            print("Error: Detected hazard and was not able to select intersection type")

        cur_angle += turn_ang
        turnPi(BP,turn_ang)
        map.cur_direc = Dir((map.cur_direc.value + turn_ang // 90) % 4)  # change current direction property based on turn_ang

        return cur_angle
    except Exception as error: 
        print("handleJunction:",error)
    except KeyboardInterrupt:
        stop(BP)
        map.pushInfo()

def exitJunction(map, set_dists, cur_angle, speed, bfr_dist, kp):
    try:
        act_dists = np.multiply(getUltras(BP), cos(radians(gyroVal(BP) - cur_angle)))
        BP.offset_motor_encoder(BP.PORT_C,BP.get_motor_encoder(BP.PORT_C))
        while BP.get_motor_encoder(BP.PORT_C) > -(35 * (360/(7* pi))) and (act_dists[0] <= set_dists[0] or act_dists[1] >= set_dists[1] + bfr_dist or act_dists[2] >= set_dists[2] + bfr_dist):
            if act_dists[1] < set_dists[1] + bfr_dist:
                errors = np.subtract(set_dists, act_dists)
                outputs  = np.multiply(kp, errors)

                # apprch right wall --> (-) error --> subtr (-) error to right w (^ spd) & add (-) error to left w (v spd) 
                if(outputs[1] >= 0):
                    setSpeed(BP, speed + outputs[1], speed) 
                # apprch left wall --> (+) error --> subtr (+) error to right w (v spd) & add (+) error to left w (^ spd)
                else:
                    setSpeed(BP, speed, speed - outputs[1])
            elif act_dists[2] < set_dists[2] + bfr_dist:
                errors = np.subtract(set_dists, act_dists)
                outputs  = np.multiply(kp, errors)

                # apprch right wall --> (+) error --> add (+) error to right w (^ spd) & subtr (+) error to left w (v spd) 
                if(outputs[2] >= 0):
                    setSpeed(BP, speed, speed + outputs[2]) 
                # apprch left wall --> (-) error --> add (-) error to right w (v spd) & subtr (-) error to left w (^ spd)
                else:
                    setSpeed(BP, speed - outputs[2], speed)  
            else:
                setSpeed(BP,speed,speed)
            act_dists = np.multiply(getUltras(BP), cos(radians(gyroVal(BP) - cur_angle)))
            time.sleep(.1)
        map.updateLocation()
        print("Update loc -- exiting junction")
    except Exception as error: 
        print("exitJunction:",error)
    except KeyboardInterrupt:
        stop(BP)
        map.pushInfo()

def mapMaze(BP, map, imu_calib,speed,set_dists,direc = Dir.UP,kp = .4,ki = .01,bfr_dist = 20,gyro_kp = .2,gyro_ki = 0.0,sensor = Sensor.LEFT):
    '''set_dists = [front sensor stop dist, left sensor set pt, right senor set pt]'''
    try:
        cur_angle = gyroVal(BP)
        act_dists = np.multiply(getUltras(BP), cos(radians(gyroVal(BP) - cur_angle)))
        
        while True:
            # sweep to parallel with wall
            delt_ang = parallelToWall(BP, cur_angle, dtheta=30, sweep_spd = 2, sensor = sensor, dt = .05)

            printGyroVal(BP)
            print("delt_ang:",delt_ang)
            cur_angle += delt_ang
            
            cur_angle, act_dists = wallGuide(map, cur_angle, act_dists, set_dists, bfr_dist, sensor, speed, kp, ki, gyro_kp, gyro_ki)

            #travel extra distance to confirm robot is in junction  and confirm junction type
            turnPi(BP,cur_angle - gyroVal(BP))
            speedControl(BP,imu_calib,speed,15)
            act_dists = np.multiply(getUltras(BP), cos(radians(gyroVal(BP) - cur_angle))) 
            print("junction case reached\nfront: %d | left: %d | right: %d" % (act_dists[0],act_dists[1],act_dists[2]))

            if not (act_dists[0] <= set_dists[0] + bfr_dist and act_dists[1] <= set_dists[1] + bfr_dist and act_dists[2] <= set_dists[2] + bfr_dist): 
                map.updateLocation()
                print("Update loc -- junction loc add")

            #check for junction cases
            cur_angle = handleJunction(map, set_dists, cur_angle, act_dists[0], act_dists[1], act_dists[2], bfr_dist) 

            # drive forward until exit junction (walls on each side and open ahead)
            exitJunction(map, set_dists, cur_angle, speed, bfr_dist, kp)

            # out of maze if traveled more than 35 cm to exit junction
            if BP.get_motor_encoder(BP.PORT_C) <= -(35 * (360/(7* pi))):
                setSpeed(BP,0,0)
                print("Exited maze")
                break

            # travel extra distance to ensure robot is between walls prior to sweep
            speedControl(BP,imu_calib,speed,5)
            time.sleep(.01)
        map.pushInfo()
           
    except Exception as error: 
        print("mapMaze:",error)
    except KeyboardInterrupt:
        stop(BP)
        map.pushInfo()

def mazeNav(BP,imu_calib,speed,set_dists,kp = .4,ki = .02,bfr_dist = 15,sensor = Sensor.LEFT,gyro_kp = .2,gyro_ki = 0.0):
    '''set_dists = [front sensor stop dist, left sensor set pt, right senor set pt]'''
    try:
        errors = [-1,1,1]
        errors_p = [0,0,0]
        integs = [0,0,0]
        dt = .2            #loop iteration ~= .15 + .05 sleep
        cur_angle = gyroVal(BP)
        act_dists = np.multiply(getUltras(BP), cos(radians(gyroVal(BP) - cur_angle)))
        
        while True:
            # sweep to parallel with wall
            dead_band_dist = .5
            delt_ang = parallelToWall(BP, cur_angle, dtheta=30, sweep_spd = 2, sensor = sensor, dt = .05)
            printGyroVal(BP)
            print("delt_ang:",delt_ang)
            cur_angle += delt_ang
            
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
                    
                    turnPi(BP,cur_angle - gyroVal(BP))

                    gyro_error = -1
                    gyro_error_p = 0
                    gyro_integ = 0
                    gyro_dt = .1

                    dps = (speed * (360/(7* pi)))

                    print("gyro PID init")
                    while abs(act_dists[1] - set_dists[1]) < dead_band_dist and act_dists[0] > set_dists[0] and act_dists[2] < set_dists[2] + bfr_dist:
                        gyro_error = cur_angle - gyroVal(BP)                         #error = system (gyro) dev from desired state (target_deg)
                        gyro_integ = gyro_integ + (gyro_dt * (gyro_error + gyro_error_p)/2)  #integral feedback (trapez approx)
                        gyro_output = gyro_kp * (gyro_error) + gyro_ki * (gyro_integ)        #PI feedback response
                        gyro_error_p = gyro_error
                        
                        BP.set_motor_dps(BP.PORT_C, dps + gyro_output)   
                        BP.set_motor_dps(BP.PORT_B, dps - gyro_output) 
                        act_dists = np.multiply(getUltras(BP), cos(radians(gyroVal(BP) - cur_angle)))
 
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

                act_dists = np.multiply(getUltras(BP), cos(radians(gyroVal(BP) - cur_angle)))

                time.sleep(.05)
            
            setSpeed(BP,0,0)
            print("junction case reached")
            print("front: %d | left: %d | right: %d" % (act_dists[0],act_dists[1],act_dists[2]))
            turnPi(BP,cur_angle - gyroVal(BP))

            #check for junction cases
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

            speedControl(BP,imu_calib,speed,10 + 10)
            cur_angle += turn_ang
            turnPi(BP,turn_ang)

            #drive forward until walls on each side and path ahead (normal)
            act_dists = np.multiply(getUltras(BP), cos(radians(gyroVal(BP) - cur_angle)))
            while act_dists[0] <= set_dists[0] or act_dists[1] >= set_dists[1] + bfr_dist or act_dists[2] >= set_dists[2] + bfr_dist:
                setSpeed(BP,speed,speed)
                act_dists = np.multiply(getUltras(BP), cos(radians(gyroVal(BP) - cur_angle)))
                time.sleep(.1)
            speedControl(BP,imu_calib,speed,5 + 8)

    except Exception as error: 
        print("mazeNav:",error)
    except KeyboardInterrupt:
        stop(BP)

def navPointsInSeq(BP,imu_calib,speed,points,length_conv = 40):
    try:
        for x in range(len(points) - 1):
            pt_2_pt(BP, imu_calib, speed, points[x], points[x+1],length_conv)

    except Exception as error: 
        print("navPointsInSeq:",error)
    except KeyboardInterrupt:
        stop(BP)

def pocTasks(BP,imu_calib,task_num,map = None):
    ### POC 1: Naviage with walls ###
    if task_num == 1 or task_num == 12:
        set_dists = [35,11,11]
        mazeNav(BP,imu_calib,10,set_dists, kp=.5, ki=0.00 ,sensor=Sensor.LEFT)

    ### POC 2: Point turning ###
    elif task_num == 2:
        turnPi(BP,11)

    ### POC 3: Avoiding hazards ###
    elif task_num == 3:
        pt_2_pt(BP,imu_calib,15,(0,0),(3,3),40,Hazard.CHECK_HAZARDS,[113.14,42,60])
        #                          (15 * 1.65) less than dist from pts ^     

    ### POC 4: Point to point navigation ###
    elif task_num == 4:
        pts = [(0,0),(2,2),(3,1),(4,3),(0,0)]
        navPointsInSeq(BP,imu_calib,15,pts)

    ### POC 5: Mapping hallway ###
    elif task_num == 5 or task_num == 56:
        set_dists = [35,11,11]
        mapMaze(BP, map, imu_calib,10,set_dists,direc=Dir.UP, kp=.5, ki=0.00 ,sensor=Sensor.LEFT)

    ### POC 3/4: Point to point with hazards ###
    elif task_num == 34:
        pts = [(0,0),(0,2),(2,2),(2,3),(0,3),(2,3),(2,2),(4,2),(4,3),(0,4),(0,0)]
        navPointsInSeq(BP,imu_calib,15,pts)
    
def createMap(direc):
    origin = input("Enter origin coordinates separted by space: ").split()
    origin = [int(i) for i in origin]
    map_num = input("Enter map number: ")
    return Map(origin,int(map_num),direc=direc)

if __name__ == '__main__':
    BP = brickpi3.BrickPi3()
    imu_calib = calibrate(BP)

    ############ Calibration Tests ###############
    # gyroTest(BP)
    # ultrasTest(BP)
    # imuMagTest()
    # irTest()
    # speedControl(BP,imu_calib,12,200)

    # x_mag, pos = speedControl(BP, imu_calib, 5, 300, haz_mode=Hazard.CHECK_HAZARDS)
    # print("Hazard magnitude",x_mag)
    # print("Distance traveled",pos)

    # delt_ang = parallelToWall(BP, 0, dtheta=30, sweep_spd = 2, sensor = Sensor.LEFT, dt = .05)
    #########################################

    set_dists = [20,10,10]
    maze_map = createMap(Dir.UP)
    
    mapMaze(BP, maze_map, imu_calib, 10, set_dists, direc=Dir.UP, kp=.5, ki=0.00 ,sensor=Sensor.LEFT)
    cargoRelease(BP, imu_calib)
