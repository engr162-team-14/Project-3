import brickpi3  
import grovepi 
import math
import time

def stop(BP):
    BP.set_motor_dps(BP.PORT_C, 0)   
    BP.set_motor_dps(BP.PORT_B, 0)
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))

    BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D))
    BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
               
    print("stopped")

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

def speedControl(BP,speed,distance):
    try:
        pos = 0  

        dps = -speed * (360/(7* math.pi))
        while distance >= pos:
            start_time = time.time()

            BP.set_motor_dps(BP.PORT_C + BP.PORT_B, dps)
            print("Motor A: %6d  B: %6d  C: %6d  D: %6d pos: %f" %
                (BP.get_motor_encoder(BP.PORT_A), BP.get_motor_encoder(BP.PORT_B),
                BP.get_motor_encoder(BP.PORT_C),BP.get_motor_encoder(BP.PORT_D),pos))	
            time.sleep(.015)
            pos += abs(speed) * (time.time() - start_time)

        stop(BP)
    except Exception as error: 
        print("speedControl:",error)
    except KeyboardInterrupt:
        stop(BP)

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