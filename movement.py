import brickpi3  
import grovepi 
import math


def gyroTest(BP):
    try:
        while True:
            rot = BP.get_sensor(BP.PORT_1)
            print(rot)

    except Exception as error: 
        print("gyroTest:",error)
    except KeyboardInterrupt:
        stop(BP)

def gyroVal(BP):
    snsr = (BP.get_sensor(BP.PORT_1))
    snsr = snsr[0]
    print("Gyro (sensor): "+str(snsr))
    return snsr


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