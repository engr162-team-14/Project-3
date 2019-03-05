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


def gyroCalib(BP):
    try:
        BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)  #set BP port 1 to Gyro
        print("calibrating gyroscope..")
        while True:
            try:
                print(BP.get_sensor(BP.PORT_1))
                print("success")
                return BP.get_sensor(BP.PORT_1)

            except Exception:
                pass
                        
            time.sleep(0.02)  

    except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
        BP.reset_all() 

def gyroVal(BP):
    snsr = (BP.get_sensor(BP.PORT_1))
    snsr = snsr[0]
    print("Gyro (sensor): "+str(snsr))
    return snsr

def gyroTest(BP):
    try:
        while True:
            rot = BP.get_sensor(BP.PORT_1)
            print(rot)

    except Exception as error: 
        print("gyroTest:",error)

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

def IMUMagnet():
    try:
        mpu9250 = MPU9250()
        while True:
            mag = mpu9250.readMagnet()
            mag_x = mag['x']
            mag_y = mag['y']
            mag_z = mag['z']
            return 

            time.sleep(.25)
    except Exception as error: 
        print("IMUTest:",error)

def IMUTest():
    try:
        mpu9250 = MPU9250()
        while True:
            gyro = mpu9250.readGyro()
            print(gyro['x'])

    except Exception as error: 
        print("IMUTest:",error)

def getUltras(port_f=0,port_l=0,port_r=0):
    return [grovepi.ultrasonicRead(port_f),grovepi.ultrasonicRead(port_l),grovepi.ultrasonicRead(port_r)]

#filtering Ultras

#ultra testing

    