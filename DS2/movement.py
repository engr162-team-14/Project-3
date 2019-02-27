import brickpi3  
import grovepi 
import math
import time
import IMUfilter

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

def IMUTest():
    try:
        while True:
            out = IMUfilter.imuOut()
            print(out)

    except Exception as error: 
        print("IMUTest:",error)
    except KeyboardInterrupt:
        stop(BP)

def balance(BP,KP,KI,KD):
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
            target_pos = IMUfilter.getGyroVals()          # get current x,y,z axis gyro pos

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
