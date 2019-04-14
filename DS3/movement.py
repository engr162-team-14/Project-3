import brickpi3  
import grovepi 
import math
import time
from sys import maxsize

from sensors import lightCalib
from sensors import lightTest
from sensors import lightVal
from sensors import imuGyroFiltered

import numpy as np

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

def speedControl(BP,speed,distance):
    try:
        dt = .08
        pos = 0

        while distance > pos:
            start_time = time.time()

            setSpeed(BP,speed,speed)

            time.sleep(dt)
            pos += abs(speed) * (time.time() - start_time)

        setSpeed(BP,0,0)
    except Exception as error: 
        print("speedControl:",error)
    except KeyboardInterrupt:
        stop(BP)

def speedControlScan(BP,speed,distance):
    try:
        dt = 1/speed
        pos = 0
        row_data = np.zeros(25)
        col_idx = 0
        while distance > pos:
            start_time = time.time()

            setSpeed(BP,speed,speed)
            row_data[col_idx] = lightVal()
            col_idx += 1

            time.sleep(dt)
            pos += abs(speed) * (time.time() - start_time)

        setSpeed(BP,0,0)
        return row_data
    except Exception as error: 
        print("speedControl:",error)
    except KeyboardInterrupt:
        stop(BP)

def turnPi(BP,deg,imu_calib,kp = .15,ki = 0.0, dead_band = 1):
    try:
        setSpeed(BP,0,0)
        axis = 2
        target_deg = imuGyroFiltered(imu_calib)[axis] + deg
        error = maxsize
        error_p = 0
        integ = 0
        dt = .1
        
        while abs(error) > dead_band:
            error = target_deg - (dt * imuGyroFiltered(imu_calib)[axis])     #error - system (gyro) dev from desired state (target_deg)
            integ = integ + (dt * (error + error_p)/2)                       #integral feedback (trapez approx)
            output = kp * (error) + ki * (integ)                             #PI feedback response
            error_p = error
            
            setSpeed(BP,output,-output)  
            time.sleep(dt)
                
        setSpeed(BP,0,0)


    except Exception as error: 
        print("turnPi:",error)
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

def pt2Pt (BP, speed, pt_1, pt_2, imu_calib, length_conv = 1):
    try:
        pt_1 = [d * length_conv for d in pt_1]
        pt_2 = [d * length_conv for d in pt_2]

        distance = getDistance(pt_1[0], pt_1[1], pt_2[0], pt_2[1])
        angle = getAngle(pt_1[0], pt_1[1], pt_2[0], pt_2[1])

        turnPi(BP, angle, imu_calib)
        speedControl(BP,speed,distance)
            
        turnPi(BP, -angle, imu_calib)
    except Exception as error: 
        print("pt2Pt",error)
    except KeyboardInterrupt:
        stop(BP) 

def pt2PtScan (BP, speed, pt_1, pt_2, imu_calib, length_conv = 1):
    try:
        pt_1 = [d * length_conv for d in pt_1]
        pt_2 = [d * length_conv for d in pt_2]

        distance = getDistance(pt_1[0], pt_1[1], pt_2[0], pt_2[1])
        angle = getAngle(pt_1[0], pt_1[1], pt_2[0], pt_2[1])

        turnPi(BP, angle, imu_calib)
        row_data = speedControlScan(BP, speed, distance)
            
        turnPi(BP, -angle, imu_calib)

        return row_data
    except Exception as error: 
        print("pt2PtScan",error)
    except KeyboardInterrupt:
        stop(BP)  
