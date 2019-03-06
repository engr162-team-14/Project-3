import brickpi3  
import grovepi 
import math
import time
from enum import Enum, auto

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

class Hazard(Enum):
    NO_HAZARDS = auto()
    CHECK_HAZARDS = auto()

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
        if drc >= 0:
            dps_l = (speed_l * (360/(7* math.pi)))
            dps_r = (speed_r * (360/(7* math.pi)))         
            
            BP.set_motor_dps(BP.PORT_B, dps_l)   
            BP.set_motor_dps(BP.PORT_C, dps_r)

        else:
            dps_l = -(speed_l * (360/(7* math.pi))) 
            dps_r = -(speed_r * (360/(7* math.pi)))         

            BP.set_motor_dps(BP.PORT_C, dps_l)   
            BP.set_motor_dps(BP.PORT_B, dps_r)

    except Exception as error: 
        print("setSpeed:",error)
    except KeyboardInterrupt:
        stop(BP)

def speedControl(BP,imu_calib,speed,distance,haz_mode = Hazard.NO_HAZARDS,pos = 0,ir_thresh = 30,mag_thresh = 30):
    try:
        while distance >= pos:
            start_time = time.time()
            
            if haz_mode == Hazard.CHECK_HAZARDS and imuMagFiltered(imu_calib) >= mag_thresh and irVal() >= ir_thresh:
                return pos
            setSpeed(BP,speed,speed)
            # print("Motor A: %6d  B: %6d  C: %6d  D: %6d pos: %f" %
            #     (BP.get_motor_encoder(BP.PORT_A), BP.get_motor_encoder(BP.PORT_B),
            #     BP.get_motor_encoder(BP.PORT_C),BP.get_motor_encoder(BP.PORT_D),pos))	
            time.sleep(.05)
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

def turnPi(BP,deg,kp = .7,ki = .3):
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

def turnPiAbs(BP,deg,kp = .7,ki = .3):
    try:
        error = -1
        error_p = 0
        integ = 0
        dt = .1
        
        while  error != 0:
            error = deg - gyroVal(BP)            #error - system (gyro) dev from desired state (target_deg)
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
        angle = math.atan(vecj/veci)
        
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
        
def pt_2_pt (BP, imu_calib, speed, angle, distance):
    try:
        turnPiAbs(BP, angle, .3, .7)
        
        speedControl(BP, imu_calib, speed,distance)
    except Exception as error: 
        print("pt_2_pt",error)
    except KeyboardInterrupt:
        stop(BP)  
     
def pt_2_pt2 (BP,imu_calib, x1, x2, y1, y2):
    '''funtion navegates the robot from one point (x1, y1) to another point (x2, y2) using the linear components '''
    try:
        veci = x2 - x1
        vecj = y2 - y1
        if(veci < 0):
            turnPiAbs(BP, 90)
            speedControl(BP, imu_calib, 6, abs(veci))
            if(vecj < 0):
                turnPiAbs(BP, 180)
                speedControl(BP, imu_calib, 6, abs(vecj))
            else:
                turnPiAbs(BP, 0)
                speedControl(BP, imu_calib, 6, abs(vecj))
        else:
            turnPiAbs(BP, 270)
            speedControl(BP, imu_calib, 6, abs(veci))
            if(vecj < 0):
                turnPiAbs(BP, 180)
                speedControl(BP, imu_calib, 6, abs(vecj))
            else:
                turnPiAbs(BP, 0)
                speedControl(BP, imu_calib, 6, abs(vecj))
    except Exception as error: 
        print("pt_2_pt2",error)
    except KeyboardInterrupt:
        stop(BP)

def hazardLocate(BP):
    try:
        while True:
            sensors = irVal()
            if sensors[0] >= 15:
                pass
    except Exception as error: 
        print("hazardLocate",error)
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