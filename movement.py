import brickpi3  
import grovepi 
import math
import time
from sys import maxsize
from enum import Enum

from sensors import gyroCalib
from sensors import gyroVal
from sensors import gyroTest
from sensors import leftUltraCalib
from sensors import leftUltraVal
from sensors import leftUltraTest
from sensors import getUltras
from sensors import imuCalib
from sensors import imuFiltered
from sensors import imuGyroFiltered
from sensors import imuGyroTest
from sensors import imuMagFiltered
from sensors import imuMagTest
from sensors import irCalib
from sensors import irTest
from sensors import irVal
from sensors import hazardCheck

class Sensor(Enum):
    FRONT = 0
    LEFT = 1
    RIGHT = 2

class Hazard(Enum):
    NO_HAZARDS = 0
    CHECK_HAZARDS = 1

def stop(BP):
    BP.set_motor_dps(BP.PORT_B, 0)
    BP.set_motor_dps(BP.PORT_C, 0)   
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))

    BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D))
    BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
               
    print("stopped")

def setSpeed(BP,speed_l,speed_r,drc = 0):
    try:
        #print(speed_l," ",speed_r)
        d = 6.85
        if drc >= 0:
            dps_l = (speed_l * (360/(d* math.pi)))
            dps_r = (speed_r * (360/(d* math.pi)))         
            
        else:
            dps_l = -(speed_l * (360/(d* math.pi))) 
            dps_r = -(speed_r * (360/(d* math.pi)))         

        BP.set_motor_dps(BP.PORT_C, dps_l)   
        BP.set_motor_dps(BP.PORT_B, dps_r)

    except Exception as error: 
        print("setSpeed:",error)
    except KeyboardInterrupt:
        stop(BP)

def setSpeedStraight(BP,speed_l,speed_r,drc = 0,kp = .2,ki = .025):
    try:
        #print(speed_l," ",speed_r)
        if drc >= 0:
            dps_l = (speed_l * (360/(7* math.pi)))
            dps_r = (speed_r * (360/(7* math.pi)))         
            
        else:
            dps_l = -(speed_l * (360/(7* math.pi))) 
            dps_r = -(speed_r * (360/(7* math.pi)))         

        if speed_l != speed_r or speed_l == speed_r == 0:
            BP.set_motor_dps(BP.PORT_C, dps_l)   
            BP.set_motor_dps(BP.PORT_B, dps_r)
        else:
            eq_deg = gyroVal(BP)
            error = -1
            error_p = 0
            integ = 0
            dt = .1
            
            while True:
                error = eq_deg - gyroVal(BP)                #error = system (gyro) dev from desired state (target_deg)
                integ = integ + (dt * (error + error_p)/2)  #integral feedback (trapez approx)
                output = kp * (error) + ki * (integ)        #PI feedback response
                error_p = error
                
                BP.set_motor_dps(BP.PORT_C, dps_l + output)   
                BP.set_motor_dps(BP.PORT_B, dps_r - output)  
                time.sleep(dt)

    except Exception as error: 
        print("setSpeedStraight:",error)
    except KeyboardInterrupt:
        stop(BP)

def speedControl(BP,imu_calib,speed,distance,kp = .2,ki = .025,pos = 0,haz_mode = Hazard.NO_HAZARDS):
    try:
        eq_deg = gyroVal(BP)
        error = -1
        error_p = 0
        integ = 0
        dt = .08

        while distance > pos:
            start_time = time.time()

            #if looking for hazards and there is 
            if haz_mode == Hazard.CHECK_HAZARDS and hazardCheck(imu_calib):
                setSpeed(BP,0,0)
                return pos

            error = eq_deg - gyroVal(BP)                #error = system (gyro) dev from desired state (target_deg)
            integ = integ + (dt * (error + error_p)/2)  #integral feedback (trapez approx)
            output = kp * (error) + ki * (integ)        #PI feedback response
            error_p = error
                
            setSpeed(BP,speed + output,speed - output)
            time.sleep(dt)

            pos += abs(speed) * (time.time() - start_time)

        setSpeed(BP,0,0)
        # ***delete later vvvvvvvvvvvv***
        if haz_mode == Hazard.CHECK_HAZARDS:
            return pos
    except Exception as error: 
        print("speedControl:",error)
    except KeyboardInterrupt:
        stop(BP)

def turnPi(BP,deg,kp = .15,ki = 0.0, dead_band = 1):
    try:
        setSpeed(BP,0,0)
        target_deg = gyroVal(BP) + deg
        error = maxsize
        error_p = 0
        integ = 0
        dt = .1
        
        while abs(error) > dead_band:
            error = target_deg - gyroVal(BP)            #error - system (gyro) dev from desired state (target_deg)
            integ = integ + (dt * (error + error_p)/2)  #integral feedback (trapez approx)
            output = kp * (error) + ki * (integ)        #PI feedback response
            error_p = error
            
            setSpeed(BP,output,-output)  
            time.sleep(dt)
                
        setSpeed(BP,0,0)
    except Exception as error: 
        print("turn_pi:",error)
    except KeyboardInterrupt:
        stop(BP)

def turnPiAbs(BP,deg,kp = .2,ki = .025):
    try:
        error = -1
        error_p = 0
        integ = 0
        dt = .1
        
        while  abs(error) > 1:
            error = deg - gyroVal(BP)                   #error - system (gyro) dev from desired state (target_deg)
            integ = integ + (dt * (error + error_p)/2)  #integral feedback (trapez approx)
            output = kp * (error) + ki * (integ)        #PI feedback response
            error_p = error
            
            setSpeed(BP,output,-output)  
            time.sleep(dt)
                
        setSpeed(BP,0,0)
    except Exception as error: 
        print("turn_pi:",error)
    except KeyboardInterrupt:
        stop(BP)

def parallelToWall(BP, init_ang, dtheta = 30, sweep_spd = 2, sensor = Sensor.LEFT, dt = .05):
    try:
        min_dist = maxsize
        targ_angle = init_ang
        
        cur_ang = gyroVal(BP)
        cur_dist = leftUltraVal(BP)
        if sensor == Sensor.LEFT:
            while cur_ang <= (init_ang + dtheta):
                print("C || cur_ang: %f | init_ang: %f | target_angle: %f | cur_dist: %f | min_dist: %f" \
                    % (cur_ang,init_ang,targ_angle,cur_dist,min_dist))
                if cur_dist >= 3 and cur_dist < min_dist:
                    min_dist = cur_dist
                    targ_angle = cur_ang
                setSpeed(BP,sweep_spd,-sweep_spd)

                cur_ang = gyroVal(BP)
                cur_dist = leftUltraVal(BP)
                time.sleep(dt)
            setSpeed(BP,0,0)

            cur_ang = gyroVal(BP)
            cur_dist = leftUltraVal(BP)
            while cur_ang >= (init_ang - dtheta):
                print("CC || cur_ang: %f | init_ang: %f | target_angle: %f | cur_dist: %f | min_dist: %f" \
                    % (cur_ang,init_ang,targ_angle,cur_dist,min_dist))
                if cur_dist >= 3 and cur_dist < min_dist:
                    min_dist = cur_dist
                    targ_angle = cur_ang
                setSpeed(BP,-sweep_spd,sweep_spd)

                cur_ang = gyroVal(BP)
                cur_dist = leftUltraVal(BP)
                time.sleep(dt)
            setSpeed(BP,0,0)
        
        turnPi(BP, targ_angle - gyroVal(BP),.075,0.0,0)

        return targ_angle - init_ang

    except Exception as error: 
        print("parallelToWall",error)
    except KeyboardInterrupt:
        stop(BP) 

def parallelToWallDuo(BP, init_ang, dtheta = 30, sweep_spd = 2, sensor = Sensor.LEFT, dt = .05):
    try:
        min_dist = maxsize
        targ_angle = init_ang
        
        cur_ang = gyroVal(BP)
        cur_dist = getUltras(BP)
        if sensor == Sensor.LEFT:
            while cur_ang <= (init_ang + dtheta):
                print("C || cur_ang: %f | init_ang: %f | target_angle: %f | cur_dist: %f | min_dist: %f" \
                    % (cur_ang,init_ang,targ_angle,cur_dist[1] + cur_dist[2],min_dist))
                if cur_dist[1] >= 3 and cur_dist[1] + cur_dist[2] < min_dist:
                    min_dist = cur_dist[1] + cur_dist[2]
                    targ_angle = cur_ang
                setSpeed(BP,sweep_spd,-sweep_spd)

                cur_ang = gyroVal(BP)
                cur_dist = getUltras(BP)
                time.sleep(dt)
            setSpeed(BP,0,0)

            cur_ang = gyroVal(BP)
            cur_dist = getUltras(BP)
            while cur_ang >= (init_ang - dtheta):
                print("CC || cur_ang: %f | init_ang: %f | target_angle: %f | cur_dist: %f | min_dist: %f" \
                    % (cur_ang,init_ang,targ_angle,cur_dist[1] + cur_dist[2],min_dist))
                if cur_dist[1] >= 3 and cur_dist[1] + cur_dist[2] < min_dist:
                    min_dist = cur_dist[1] + cur_dist[2]
                    targ_angle = cur_ang
                setSpeed(BP,-sweep_spd,sweep_spd)

                cur_ang = gyroVal(BP)
                cur_dist = getUltras(BP)
                time.sleep(dt)
            setSpeed(BP,0,0)
        
        turnPi(BP, targ_angle - gyroVal(BP),.1,0.0,0)

        return targ_angle - init_ang

    except Exception as error: 
        print("parallelToWall",error)
    except KeyboardInterrupt:
        stop(BP) 


def getAngle (x1, y1, x2, y2):
    try:
        if(y2 == y1):
            if(x2 > x1):
                angle = 90
            elif(x2 < x1):
                angle = -90
            else:
                angle = 0
        else:
            veci = x2 - x1
            vecj = y2 - y1
            angle = math.degrees(math.atan(veci/vecj))

            if(veci < 0):
                if(vecj == 0):
                    angle = -90
                elif(vecj < 0):
                    angle = angle - 180
                else:
                    angle = angle
            elif(veci == 0):
                if(vecj > 0):
                    angle = 0
                else:
                    angle = 180
            else:
                if(vecj == 0):
                    angle = 90
                elif(vecj < 0):
                    angle = angle + 180
                else:
                    angle = angle
        return angle

    except Exception as error: 
        print("angle",error)

def getDistance (x1, y1, x2, y2):
    try:
	    veci = x2 - x1
	    vecj = y2 - y1
	    mag = math.sqrt(math.pow(veci, 2) + math.pow(vecj, 2))

	    return mag
    except Exception as error: 
        print("distance",error)
        
def pt_2_pt (BP, imu_calib, speed, pt_1, pt_2, length_conv = 40, haz_mode = Hazard.NO_HAZARDS, est_avd_dists = [50,18,30]):
    '''est_avd_dists = [dist to hazard zone, dist needed to traverse horiz, dist needed to traverse vertically]'''
    try:
        pt_1 = [d * length_conv for d in pt_1]
        pt_2 = [d * length_conv for d in pt_2]

        distance = getDistance(pt_1[0], pt_1[1], pt_2[0], pt_2[1])
        angle = getAngle(pt_1[0], pt_1[1], pt_2[0], pt_2[1])
        turnPi(BP, angle)
        if haz_mode == Hazard.NO_HAZARDS:
            speedControl(BP, imu_calib, speed, distance, haz_mode = haz_mode)
        else:
            pos = speedControl(BP, imu_calib,speed,est_avd_dists[0], haz_mode = haz_mode)
            
            turnPi(BP,-90)
            speedControl(BP, imu_calib,speed,est_avd_dists[1], haz_mode = haz_mode)
        
            turnPi(BP,90)
            speedControl(BP, imu_calib,speed,est_avd_dists[2], haz_mode = haz_mode)
            pos += est_avd_dists[2]
            
            turnPi(BP,90)
            speedControl(BP, imu_calib,speed,est_avd_dists[1], haz_mode = haz_mode)

            turnPi(BP,-90)
            speedControl(BP, imu_calib,speed, distance, pos = pos, haz_mode = haz_mode)
            
        turnPi(BP, -angle)
        input("Hit enter to travel to next point")
    except Exception as error: 
        print("pt_2_pt",error)
    except KeyboardInterrupt:
        stop(BP)  

def cargoRelease(BP, imu_calib):
    try:
        ultraVals = getUltras(BP)
        if ultraVals[0] >= 40 and ultraVals[1] >= 40 and ultraVals[2] >= 40:
            print("I have arrived")
            val = BP.get_motor_encoder(BP.PORT_A)
            theta = val + 2300
            BP.set_motor_position(BP.PORT_A, theta)
            time.sleep(1.5)
            speedControl(BP, imu_calib ,5,10)
            BP.set_motor_position(BP.PORT_A, val)
        else:
            print("I have not arrived")
            print(ultraVals)
    except Exception as error:
        print("cargoRelease: ", error)
    except KeyboardInterrupt:
        stop(BP)


################ PLEASE MOVE THIS TO SENSORS -- TALK TO ME ASAP IF POSSIBLE ########################
def hazardDist(imu_calib, mode, x, y):
    try:
        if mode == 1:
            print('Ima workin on this')
        if mode == 2:
            print('Ima workin on this')
    except Exception as error:
        print("hazardDist: ", error)
            
def hazardCheck(imu_calib, ir_thresh = 130,magx_thresh = 30, magy_thresh = 115):
    try:
        haz = 0
        mode = 0
        while True:
            ir_val = irVal()
            mag_val = imuMag()
            if ir_val[0] >= ir_thresh or ir_val[1] >= ir_thresh:
                setSpeed(BP,0,0)
                time.sleep(.5)
                haz_ir_val = irVal()
                haz = 1

                mode = 1
            if mag_val[1] >= magy_thresh:
                setSpeed(BP, 0, 0)
                time.sleep(.5)
                haz_mag_val = imuMag()
                haz = 2
                #Between 0 and 5 cm
                if haz_mag_val[0] >= 0 and haz_mag_val[0] <= magx_thresh:
                    mode = 2
                #Between -5 and 0 cm
                elif haz_mag_val[0] >= -1 * magx_thresh and haz_mag_val[0] < 0:
                    mode = 3
                #Between 5 and 10 cm
                elif haz_mag_val[0] > magx_thresh:
                    mode = 4
                #Between -10 and -5 cm
                elif haz_mag_val[0] < magx_thresh:
                    mode = 5
        return haz
    except Exception as error:
        print("hazardCheck: ", error) 

#################################################################################################