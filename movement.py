import brickpi3  
import grovepi 
import math
import time

from sensors import gyroCalib
from sensors import gyroVal
from sensors import gyroTest
from sensors import imuCalib
from sensors import imuFiltered
from sensors import getGyroFilterVals
from sensors import IMUTest


def stop(BP):
    BP.set_motor_dps(BP.PORT_C, 0)   
    BP.set_motor_dps(BP.PORT_B, 0)
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))

    BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D))
    BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
               
    print("stopped")

def setSpeed(BP,speed_l,speed_r,drc = 0):
    try:
        #print(speed_l," ",speed_r)
        if drc >= 0:
            dps_l = -(speed_l * (360/(7* math.pi))) 
            dps_r = -(speed_r * (360/(7* math.pi)))         
            
            BP.set_motor_dps(BP.PORT_B, dps_l)   
            BP.set_motor_dps(BP.PORT_C, dps_r)

        else:
            dps_l = (speed_l * (360/(7* math.pi))) 
            dps_r = (speed_r * (360/(7* math.pi)))         

            BP.set_motor_dps(BP.PORT_C, dps_l)   
            BP.set_motor_dps(BP.PORT_B, dps_r)

    except Exception as error: 
        print("setSpeed:",error)
    except KeyboardInterrupt:
        stop(BP)

def speedControl(BP,speed,distance):
    try:
        pos = 0  

        while distance >= pos:
            start_time = time.time()

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

def turn_simple(BP,deg):
    try:
        if deg > 0:
            while gyroVal(BP) < deg:
                setSpeed(BP,5,-5)
        else:
            while gyroVal(BP) > deg:
                 setSpeed(BP,-5,5)
        setSpeed(BP,0,0)
    except Exception as error: 
        print("turn_simple:",error)
    except KeyboardInterrupt:
        stop(BP)

def turn_pi(BP,deg,kp,ki):
    try:
        error = -1
        error_p = 0
        integ = 0
        dt = .1
        
        while error != 0:
            error = deg - gyroVal(BP)
            integ = integ + (dt * (error + error_p)/2)
            output = kp * (error) + ki * (integ)
            error_p = error
            
            if(deg > 0):
                setSpeed(BP,output,-output)
            else:
                setSpeed(BP,-output,output)
            time.sleep(dt)
                
        setSpeed(BP,0,0)
    except Exception as error: 
        print("turn_pi:",error)
    except KeyboardInterrupt:
        stop(BP)

def wall_avoid_pi(BP,min_dist,speed,k):
    try:
        while True:
            ultras_sens = []
            if ultras_sens[0] < min_dist:
                pass

    except Exception as error: 
        print("wall_avoid:",error)
    except KeyboardInterrupt:
        stop(BP)

def get_angle (x1, y1, x2, y2):
    try:
        veci = x2 - x1
        vecj = y2 - y1
        angle = math.atan(vecj/veci)
        
        return angle
    except Exception as error: 
        print("angle",error)

def get_distance (x1, y1, x2, y2):
    try:
	    veci = x2 - x1
	    vecj = y2 - y1
	    mag = math.sqrt(math.pow(veci, 2) + math.pow(vecj, 2))

	    return mag
    except Exception as error: 
        print("distance",error)
        
def pt_2_pt (BP, angle, distance):
    try:
        turn_pi(BP, angle, .3, .7)
        
        #speedControl(BP,speed,distance)
        traveled = 0
        while (distance - traveled != 0):
            dt = .1
            speed = setSpeed(BP, 6, 6)
            time.sleep(dt)
            traveled = traveled + speed * dt
    except Exception as error: 
        print("pt_2_pt",error)
    except KeyboardInterrupt:
        stop(BP)  
        
def pt_2_pt2 (BP, x1, x2, y1, y2):
    try:
        veci = x2 - x1
        vecj = y2 - y1
        if
        speedControl(BP, 6, veci)
        turn_pi(BP, )
        
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
integ = integ + (dt * error * .5)                   #triangular approx
integ = integ + (dt * (error + error_p)/2)           #trapezoidal approx
deriv = (error - error_p)/dt

output  = (kp * error) + (ki * integral)  + (kd * deriv)
error_p = error
'''