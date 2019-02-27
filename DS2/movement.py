import brickpi3  
import grovepi 
import math
import time

from IMUFilters import AvgCali
from IMUFilters import genWindow
from IMUFilters import WindowFilterDyn
from IMUFilters import KalmanFilter
from IMUFilters import FindSTD
from IMUFilters import InvGaussFilter
from MPU9250 import MPU9250

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

def imuCalib(width=1, depth=100, dly=0.01):
    try:
        #pick: 1 uses window filter, anything else uses Kalman
        mpu9250 = MPU9250()

        accelx=genWindow(width,0)#Can play with width to adjust system
        accely=genWindow(width,0)
        accelz=genWindow(width,0)
        gyrox=genWindow(width,0)#Can play with width to adjust system
        gyroy=genWindow(width,0)
        gyroz=genWindow(width,0)
        magx=genWindow(width,0)#Can play with width to adjust system
        magy=genWindow(width,0)
        magz=genWindow(width,0)
        flter=[[0.7,1.0],[0.7,1.0],[0.7,1.0],[0.7,1.0],[0.7,1.0],[0.7,1.0],[5,2],[5,2],[5,2]]
        # [r,q]Will need to play with each filter value
        # 
        # A note on Kalman filter parameters: Each of these parameters represents the process and system noise respectively.
        # In order to adjust the output of the Kalman filter, each of these parameters can be modified.  Do not set these
        # parameters to zero, because a Kalman filter actually needs a little bit of noise, or it becomes unstable and 
        # effectively useless.

        # System Noise:  Setting the system noise to a high value will make the filter less responsive to subtle changes in
        # the environment.  Practically speaking, if you tell your filter that the system is going to be very noisy, it will
        # likely assume small changes are just noise.  Telling your filter that the system will have a low amount of noise
        # will make it more "aggressive".

        # Process Noise:  Process noise is a "natural noise" that grows over time proportional to how often you are making 
        # measurements.  What it attempts to model is the fact that the states change between measurements, so there is an
        # additional uncertainty on top of any noise in the sensors.  There are advanced methods for calculating good estimates
        # process noise, but generally for this course, guessing and testing should be a decent method.

        biases=AvgCali(mpu9250,depth,dly)
        std=FindSTD(biases,mpu9250,dly)

        return [mpu9250, [accelx,accely,accelz], [gyrox,gyroy,gyroz], [magx,magy,magz], flter, biases, dly, std]

    except Exception as error: 
        print("imuCalib",error)

def imuFiltered(imu_calib,adv = True, pick = 1, count = 3):
    imu = imu_calib
    out=[0,0,0,0,0,0,0,0,0]
    state=[[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[0,0,0,0,0,0,0,0,0]]   #Estimated error (p) and measurement state (x) 

    try:
        if pick == 1:
            accel = imu["mpu"].readAccel()
            gyro = imu["mpu"].readGyro()
            mag = imu["mpu"].readMagnet()
    
            accelx=WindowFilterDyn(imu["accel"][0],imu["dly"],InvGaussFilter(adv,accel['x'],imu["biases"][0],imu["std"][0],count))
            accely=WindowFilterDyn(imu["accel"][1],imu["dly"],InvGaussFilter(adv,accel['y'],imu["biases"][1],imu["std"][1],count))
            accelz=WindowFilterDyn(imu["accel"][2],imu["dly"],InvGaussFilter(adv,accel['z'],imu["biases"][2],imu["std"][2],count))
            gyrox=WindowFilterDyn(imu["gyro"][0],imu["dly"],InvGaussFilter(adv,gyro['x'],imu["biases"][3],imu["std"][3],count))
            gyroy=WindowFilterDyn(imu["gyro"][1],imu["dly"],InvGaussFilter(adv,gyro['y'],imu["biases"][4],imu["std"][4],count))
            gyroz=WindowFilterDyn(imu["gyro"][2],imu["dly"],InvGaussFilter(adv,gyro['z'],imu["biases"][5],imu["std"][5],count))
            magx=WindowFilterDyn(imu["mag"][0],imu["dly"],InvGaussFilter(adv,mag['x'], imu["biases"][6],imu["std"][6],count))
            magy=WindowFilterDyn(imu["mag"][1],imu["dly"],InvGaussFilter(adv,mag['y'], imu["biases"][7],imu["std"][7],count))
            magz=WindowFilterDyn(imu["mag"][2],imu["dly"],InvGaussFilter(adv,mag['z'], imu["biases"][8],imu["std"][8],count))
            out[0]=accelx[0]
            out[1]=accely[0]
            out[2]=accelz[0]
            out[3]=gyrox[0]
            out[4]=gyroy[0]
            out[5]=gyroz[0]
            out[6]=magx[0]
            out[7]=magy[0]
            out[8]=magz[0]
        else:
            state=KalmanFilter(imu["mpu"],state,imu["flter"],imu["dly"])
            out[0]=InvGaussFilter(adv,state[1][0], imu["biases"][0],imu["std"][0],count)
            out[1]=InvGaussFilter(adv,state[1][1], imu["biases"][1],imu["std"][1],count)
            out[2]=InvGaussFilter(adv,state[1][2], imu["biases"][2],imu["std"][2],count)
            out[3]=InvGaussFilter(adv,state[1][3], imu["biases"][3],imu["std"][3],count)
            out[4]=InvGaussFilter(adv,state[1][4], imu["biases"][4],imu["std"][4],count)
            out[5]=InvGaussFilter(adv,state[1][5], imu["biases"][5],imu["std"][5],count)
            out[6]=InvGaussFilter(adv,state[1][6], imu["biases"][6],imu["std"][6],count)
            out[7]=InvGaussFilter(adv,state[1][7], imu["biases"][7],imu["std"][7],count)
            out[8]=InvGaussFilter(adv,state[1][8], imu["biases"][8],imu["std"][8],count)
                
        return out

    except Exception as error: 
        print("imuFiltered",error)

def getGyroFilterVals(imu_calib):
    out = imuFiltered(imu_calib)
    return [out[3],out[4],out[5]]

def IMUTest(BP):
    try:
        mpu9250 = MPU9250()
        while True:
            gyro = mpu9250.readGyro()
            print(gyro['x'])

    except Exception as error: 
        print("IMUTest:",error)
    except KeyboardInterrupt:
        stop(BP)

def balance(BP,imu_calib,KP,KI,KD):
    try:
        # KP = 0.0 # proportional control gain
        # KI = 0.0 # integral control gain
        # KD = 0.0 # derivative control gain
        dT = 0.02 # time step

        target_pos = 0
        current_pos = 0

        P = 0
        I = 0
        D = 0
        e_prev = 0
        while True:
            target_pos = getGyroFilterVals(imu_calib)          # get current x,y,z axis gyro pos

            current_pos = [BP.get_motor_encoder(BP.PORT_A),BP.get_motor_encoder(BP.PORT_B)] # get current motor position

            print("current position: x - " + str(current_pos[0]) + " | y - " + str(current_pos[1]))

            e = [target_pos[0] - current_pos[0], target_pos[1] - current_pos[1]]# error
            print("error is  x - " + str(e[0]) + " | y - " + str(e[1]))

            # set up P,I,D, terms for control inputs
            P = [KP * e[0], KP * e[1]]
            I += [KI * e[0] * dT/2, KI * e[1] * dT/2]
            D = [KD * (e[0] - e_prev)/ dT, KD * (e[1] - e_prev)/ dT]

            # control input for motor
            power_in_x = P[0] + I[0] + D[0]
            power_in_y = P[1] + I[1] + D[1]
            BP.set_motor_power(BP.PORT_A, power_in_x)
            BP.set_motor_power(BP.PORT_B, power_in_y)

            e_prev = e                                # save error for this step; needed for D
            time.sleep(dT)
    except Exception as error: 
        print("balance:",error)
    except KeyboardInterrupt:
        stop(BP)
