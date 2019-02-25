import time #import time library
import brickpi3 #import BrickPi3 drivers
import grovepi #import the GrovePi drivers
import math

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

def obstBoost(BP,ang,ang_off):
        try:                
                rot = gyroVal(BP) - ang_off   #rotation from start gyro value
                
                if rot >= ang * 3:       #designation for a hill
                        return [12,20]
                              # ^----------[speed multiplier, iterations through the loop to maintain this speed]
                elif rot >= ang:         #designation for a normal "small obstacle"
                        return [5,30]
                else:                    #no hill or small obstacle detected
                        return [1,0]
                
        except Exception as error: 
                print("obstBoost:",error)
        except KeyboardInterrupt:
                stop(BP)

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

def speedTest(BP,power):
        try:
                while True:
                        BP.set_motor_power(BP.PORT_C + BP.PORT_B, power)
                        print("Motor A: %6d  B: %6d  C: %6d  D: %6d" %
                              (BP.get_motor_encoder(BP.PORT_A), BP.get_motor_encoder(BP.PORT_B),
                               BP.get_motor_encoder(BP.PORT_C),BP.get_motor_encoder(BP.PORT_D))) 
        except Exception as error: 
                print(error)
        except KeyboardInterrupt:
                stop(BP)

def speedControlGyro(BP,speed,distance,ang,ang_off):
        '''Speed control for traversing obstacles and hills'''
        try:
                pos = 0                              
                obst = obstBoost(BP,ang,ang_off)
                while distance >= pos:
                        start_time = time.time()

                        speed_a = -(speed) * obst[0]
                        if (obst[1] == 0):
                                obst = obstBoost(BP,ang,ang_off)
                        else:
                                obst[1] -= 1
                                
                        dps = speed_a * (360/(7 * math.pi))

                        BP.set_motor_dps(BP.PORT_C + BP.PORT_B, dps)
                        print("Motor A: %6d  B: %6d  C: %6d  D: %6d pos: %f" %
                                (BP.get_motor_encoder(BP.PORT_A), BP.get_motor_encoder(BP.PORT_B),
                                BP.get_motor_encoder(BP.PORT_C),BP.get_motor_encoder(BP.PORT_D),pos))	
                        time.sleep(.02)
                        pos += abs(speed_a) * (time.time() - start_time)

                stop(BP)
        except Exception as error: 
                print("speedControlGyro:",error)
        except KeyboardInterrupt:
                stop(BP)

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

def burst(BP,t,speed_l,speed_r):
        try:
                print("BUUUURST")
                if speed_l > speed_r:
                        print(" RIGHT")
                elif speed_r > speed_l:
                        print(" LEFT")
                else:
                        print(" STRAIGHT")
                while t > 0:
                        start_time = time.time()
                        setSpeed(BP,speed_l,speed_r)
                        time.sleep(.1)
                        t -= time.time() - start_time
                
                setSpeed(BP,0,0)
        
        except Exception as error: 
                print("burstRight:",error)
        except KeyboardInterrupt:
                stop(BP) 

def dumpGrad(BP,chg,t,theta):
    try:
        BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
        ang = chg
        
        while (abs(ang) <= abs(theta)):
                BP.set_motor_position(BP.PORT_A,ang)
                ang += chg
                time.sleep(t)
   
    except Exception as error: 
        print(error)
    except KeyboardInterrupt:
        stop(BP)

def dump(BP,gate_i,gate_f,ang,ang_off):
    try:
        BP.set_motor_position(BP.PORT_A, gate_i)
        speedControlGyro(BP,-10,5,ang,ang_off)

        BP.set_motor_position(BP.PORT_A, gate_f)
        time.sleep(1.5)

        speedControlGyro(BP,25,5,ang,ang_off)
        speedControlGyro(BP,-25,15,ang,ang_off)
        BP.set_motor_position(BP.PORT_A, gate_i)
   
    except Exception as error: 
        print(error)
    except KeyboardInterrupt:
        stop(BP)

def gate(BP,theta):
    try:
        BP.set_motor_position(BP.PORT_A, theta)  
   
    except Exception as error: 
        print(error)
    except KeyboardInterrupt:
        stop(BP)

def turn(BP,speed,distance):
    try:
        BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
        BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))
        BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D))
        BP.reset_all()

        BP.set_motor_position(BP.PORT_A, 340)
        speedControl(BP,speed,distance)        
    except Exception as error: 
        print(error)
    except KeyboardInterrupt:
        stop(BP)

def collisionAvoidance(BP,port,cruise_dist):
    ultrasonic_sensor_port = port# assign ultrasonic sensor port to D4
   
    try:
        if grovepi.ultrasonicRead(ultrasonic_sensor_port) < cruise_dist:
            return 1
        else:
            return 0

    except Exception as error: 
        print(error)
    except KeyboardInterrupt:
        stop(BP)
