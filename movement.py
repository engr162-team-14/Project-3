import brickpi3  
import grovepi 
import math
import time
from enum import Enum

from sensors import gyroCalib
from sensors import gyroVal
from sensors import gyroTest
from sensors import frontUltraCalib
from sensors import frontUltraVal
from sensors import frontUltraTest
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

def setSpeed(BP,speed_l,speed_r,drc = 0,kp = .2,ki = .025):
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
                
                BP.set_motor_dps(BP.PORT_C, dps_l - output)   
                BP.set_motor_dps(BP.PORT_B, dps_r + output)  
                time.sleep(dt)

    except Exception as error: 
        print("setSpeed:",error)
    except KeyboardInterrupt:
        stop(BP)

def speedControl(BP,imu_calib,speed,distance,kp = .2,ki = .025,pos = 0,haz_mode = Hazard.NO_HAZARDS):
    try:
        dps = (speed * (360/(7* math.pi)))
        eq_deg = gyroVal(BP)
        error = -1
        error_p = 0
        integ = 0
        dt = .08

        while distance > pos:
            start_time = time.time()
            
            if haz_mode == Hazard.CHECK_HAZARDS and hazardCheck(imu_calib):
                return pos

            error = eq_deg - gyroVal(BP)                #error = system (gyro) dev from desired state (target_deg)
            integ = integ + (dt * (error + error_p)/2)  #integral feedback (trapez approx)
            output = kp * (error) + ki * (integ)        #PI feedback response
            error_p = error
                
            BP.set_motor_dps(BP.PORT_B, dps - output)   
            BP.set_motor_dps(BP.PORT_C, dps + output)  
            time.sleep(dt)

            pos += abs(speed) * (time.time() - start_time)

        setSpeed(BP,0,0)
    except Exception as error: 
        print("speedControl:",error)
    except KeyboardInterrupt:
        stop(BP)

def turnSimple(BP,deg):
    try:
        if deg > 0:
            while gyroVal(BP) < deg:
                setSpeed(BP,5,-5)   
        else:
            while gyroVal(BP) > deg:
                setSpeed(BP,5,-5)
        setSpeed(BP,0,0)
    except Exception as error: 
        print("turn_simple:",error)
    except KeyboardInterrupt:
        stop(BP)

def turnPi(BP,deg,kp = .2,ki = .025):
    try:
        target_deg = gyroVal(BP) + deg
        error = -1
        error_p = 0
        integ = 0
        dt = .1
        
        while  error != 0:
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
        
        while  -0.1 <error < 0.1:
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

def getAngle (x1, y1, x2, y2):
    try:
        veci = x2 - x1
        vecj = y2 - y1
        angle = math.atan(veci/vecj)
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
        return math.degrees(angle)
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
        
def pt_2_pt (BP, imu_calib, speed, pt_1, pt_2, length_conv = 5, haz_mode = Hazard.NO_HAZARDS, guestimates = [50,30,17]):
    try:
        distance = length_conv * getDistance(pt_1[0], pt_1[1], pt_2[0], pt_2[1])
        angle = getAngle(pt_1[0], pt_1[1], pt_2[0], pt_2[1])
        turnPi(BP, angle)
        if haz_mode == Hazard.NO_HAZARDS:
            speedControl(BP, imu_calib, speed, distance, haz_mode = haz_mode)
        else:
            pos = speedControl(BP, imu_calib,speed,guestimates[0], haz_mode = haz_mode)
            
            turnPi(BP,-90)
            speedControl(BP, imu_calib,speed,guestimates[1], haz_mode = haz_mode)
        
            turnPi(BP,90)
            speedControl(BP, imu_calib,speed,guestimates[2], haz_mode = haz_mode)
            pos += guestimates[2]
            
            turnPi(BP,90)
            speedControl(BP, imu_calib,speed,guestimates[1], haz_mode = haz_mode)

            turnPi(BP,-90)
            speedControl(BP, imu_calib,speed,distance,pos = pos, haz_mode = haz_mode)
            
        turnPi(BP, -angle)
    except Exception as error: 
        print("pt_2_pt",error)
    except KeyboardInterrupt:
        stop(BP)  

'''
def pt_2_pt_abs (BP, imu_calib, speed, pt_1, pt_2, init_ang, length_conv = 5, haz_mode = Hazard.NO_HAZARDS):
    try:
        distance = length_conv * getDistance(pt_1[0], pt_1[1], pt_2[0], pt_2[1])
        angle = init_ang - getAngle(pt_1[0], pt_1[1], pt_2[0], pt_2[1])
        turnPi(BP, angle)
        if haz_mode == Hazard.NO_HAZARDS:
            speedControl(BP, imu_calib, speed, distance, haz_mode = haz_mode)
        else:
            pos = speedControl(BP, imu_calib,speed,distance, haz_mode = haz_mode)
            # calculate new route and get there...
        turnPi(BP, -angle)
    except Exception as error: 
        print("pt_2_pt_abs",error)
    except KeyboardInterrupt:
        stop(BP)
'''
Isaac = gyroVal(BP)

def go_to_90 (BP):
    try:
        current = gyroVal(BP)
        turnDegree = 90 + Isaac - current
        return turnDegree
    except Exception as error: 
        print("go_to_90",error)
    except KeyboardInterrupt:
        stop(BP)
def go_to_180 (BP):
    try:
        current = gyroVal(BP)
        turnDegree = 180 + Isaac - current
        return turnDegree
    except Exception as error: 
        print("go_to_180",error)
    except KeyboardInterrupt:
        stop(BP)
def go_to_90_2 (BP):
    try:
        current = gyroVal(BP)
        turnDegree = -90 + Isaac - current
        return turnDegree
    except Exception as error: 
        print("go_to_90_2",error)
    except KeyboardInterrupt:
        stop(BP)
def go_to_0 (BP):
    try:
        current = gyroVal(BP)
        turnDegree = Isaac - current
        return turnDegree
    except Exception as error: 
        print("go_to_0",error)
    except KeyboardInterrupt:
        stop(BP)

def pt_2_pt2 (BP,imu_calib, x1, x2, y1, y2):
    '''funtion navegates the robot from one point (x1, y1) to another point (x2, y2) using the linear components '''
    try:
        veci = x2 - x1
        vecj = y2 - y1
        if(veci < 0):
            deg = go_to_90(BP)
            turnPi(BP, deg)
            speedControl(BP, imu_calib, 6, abs(veci))
            if(vecj < 0):
                deg = go_to_180(BP)
                turnPi(BP, deg)
                speedControl(BP, imu_calib, 6, abs(vecj))
            else:
                deg = go_to_0(BP)
                turnPi(BP, 0)
                speedControl(BP, imu_calib, 6, abs(vecj))
        else:
            deg = go_to_90_2(BP)
            turnPi(BP, deg)
            speedControl(BP, imu_calib, 6, abs(veci))
            if(vecj < 0):
                go_to_180
                turnPi(BP, deg)
                speedControl(BP, imu_calib, 6, abs(vecj))
            else:
                deg = go_to_0(BP)
                turnPi(BP, deg)
                speedControl(BP, imu_calib, 6, abs(vecj))
    except Exception as error: 
        print("pt_2_pt2",error)
    except KeyboardInterrupt:
        stop(BP)
            
                
            
       
        

'''
Things to integrate into code

--PID
kp = .3
ki = .7
kd = .1

error = set - curr
integ = integ + (dt * error * .5)                    #triangular approx
integ = integ + (dt * (error + error_p)/2)           #trapezoidal approx
deriv = (error - error_p)/dt

output  = (kp * error) + (ki * integral)  + (kd * deriv)
error_p = error
'''