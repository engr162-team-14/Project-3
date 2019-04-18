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
from mapping import State


def gyroCalib(BP):
    try:
        BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.NONE)
        BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)  #set BP port 4 to Gyro
        print("calibrating gyroscope..")
        while True:
            try:
                print(BP.get_sensor(BP.PORT_4))
                print("success")
                return BP.get_sensor(BP.PORT_4)
            except Exception:
                pass              
            time.sleep(0.02) 
    except Exception as error: 
        print("gyroCalib:",error)
    except KeyboardInterrupt:
        BP.reset_all()

def gyroVal(BP):
    try:
        snsr = (BP.get_sensor(BP.PORT_4))
        snsr = snsr[0]
        return snsr
    except Exception as error: 
        print("gyroVal:",error)

def printGyroVal(BP):
    try:
        print("gyro: %d" % gyroVal(BP))
    except Exception as error: 
        print("printGyroVal:",error)

def gyroTest(BP):
    try:
        while True:
            rot = BP.get_sensor(BP.PORT_4)
            print(rot)
    except Exception as error: 
        print("gyroTest:",error)
    except KeyboardInterrupt:
        BP.reset_all()

def leftUltraCalib(BP):
    try:
        BP.set_sensor_type(BP.PORT_3, BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)  #set BP port 3 to Left Ultra
        print("calibrating left ultrasonic sensor..")
        while True:
            try:
                print(BP.get_sensor(BP.PORT_3))
                print("success")
                return BP.get_sensor(BP.PORT_3)
            except Exception:
                pass   
            time.sleep(0.02)  
    except Exception as error: 
        print("frontUltraCalib:",error)
    except KeyboardInterrupt:
        BP.reset_all()

def leftUltraVal(BP):
    try:
        snsr = (BP.get_sensor(BP.PORT_3))
        #print("Left Ultra: "+str(snsr))
        return snsr
    except Exception as error: 
        print("frontUltraVal:",error)

def leftUltraTest(BP):
    try:
        while True:
            snsr = BP.get_sensor(BP.PORT_3)
            print(snsr)

    except Exception as error: 
        print("leftUltraTest:",error)
    except KeyboardInterrupt:
        BP.reset_all()

def getUltras(BP,port_f=3,port_r=4):
    try:
        ultras = [grovepi.ultrasonicRead(port_f),BP.get_sensor(BP.PORT_3),grovepi.ultrasonicRead(port_r)]
        # if len(x for x in ultras if x < 1000) == 3:
        #     return ultras
        # else:
        #     return None
        return ultras
    except Exception as error: 
        print("getUltras:",error)

def ultrasTest(BP):
    try:
        while True:
            print(getUltras(BP))
            time.sleep(.1)

    except Exception as error: 
        print("ultrasTest:",error)
    except KeyboardInterrupt:
        BP.reset_all()

#filtering Ultras

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

def imuGyroFiltered(imu_calib):
    try:
        out = imuFiltered(imu_calib)
        return [out[3],out[4],out[5]]
    except Exception as error: 
        print("imuGyroFiltered:",error)

def imuGyroTest():
    try:
        mpu9250 = MPU9250()
        while True:
            gyro = mpu9250.readGyro()
            print(gyro['x'])

    except Exception as error: 
        print("imuGyroTest:",error)

def imuMagFiltered(imu_calib):
    try:
        out = imuFiltered(imu_calib)
        return [out[6],out[7],out[8]]
    except Exception as error: 
        print("imuMagFiltered:",error)

def imuMagTest():
    try:
        mpu9250 = MPU9250()
        while True:
            mag = mpu9250.readMagnet()
            mag_x = mag['x']
            mag_y = mag['y']
            mag_z = mag['z']
            print("x: %f  y: %f  z: %f" % (mag_x,mag_y,mag_z))
            time.sleep(.25)
        return mag
    except Exception as error: 
        print("imuMagFiltered: ",error)

def imuMag():
    try:
        mpu9250 = MPU9250()
        while True:
            mag = mpu9250.readMagnet()
            mag_x = mag['x']
            mag_y = mag['y']
            mag_z = mag['z']
            time.sleep(.25)
        return [mag_x,mag_y,mag_z]
    except Exception as error: 
        print("imuMag: ",error)

def irCalib(pin1 = 14, pin2 = 15):
    try:
        # Pin 14 and Pin 15 are A0 Port.
        grovepi.pinMode(pin1,"INPUT")
        grovepi.pinMode(pin2,"INPUT")
    except Exception as error: 
        print("irCalib:",error)

def irTest(pin1 = 14, pin2 = 15):
    try:  
        while True:           
            sensor1_value = grovepi.analogRead(pin1)
            sensor2_value = grovepi.analogRead(pin2)
            
            print ("One = " + str(sensor1_value) + "\tTwo = " + str(sensor2_value))
            time.sleep(.1)

    except Exception as error: 
        print("irTest:",error)
		
def irVal(pin1 = 14, pin2 = 15):
    try:            
            sensor1_value = grovepi.analogRead(pin1)
            sensor2_value = grovepi.analogRead(pin2)

            return [sensor1_value, sensor2_value]
    except Exception as error: 
        print("irVal:",error)

            
def hazardCheck(imu_calib, ir_thresh = 130 ,magx_thresh = 30 , magy_thresh = 115):
    '''
    Description: Checks for hazards directly in front of robot given sensor thresholds \n 
    Return value: hazardCheck returns [ hazard_type, hazard_val ] \n
               hazard_type -- State Enum that signifies hazard type (State.HEAT, State.Mag,
                                or None if there is no hazard within ~40cm)
               hazard_val  -- Measured relative strength of hazard detected (None if hazard_type is None)
    '''
    try:
        while True:
            ir_val = irVal()
            mag_val = imuMagFiltered(imu_calib)
            if ir_val[0] >= ir_thresh or ir_val[1] >= ir_thresh:
                time.sleep(.5)
                haz_ir_val = irVal()
                if haz_ir_val[0] >= ir_thresh or haz_ir_val[1] >= ir_thresh:
                    haz = State.HEAT
                    magnitude = math.sqrt(ir_val[0]**2 + ir_val[1]**2)
                else:
                    haz = None
                    magnitude = None
            elif mag_val[1] >= magy_thresh:
                time.sleep(.5)
                haz_mag_val = imuMagFiltered(imu_calib)
                if haz_mag_val[1] >= magy_thresh:
                    haz = State.Mag
                    magnitude = math.sqrt(mag_val[0]**2 + mag_val[1]**2 + mag_val[2]**2)
                else:
                    haz = None
                    magnitude = None
            else:
                haz = None
                magnitude = None
                
        return [haz, magnitude]
    except Exception as error:
        print("hazardCheck: ", error) 
