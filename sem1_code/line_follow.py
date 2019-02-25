import time
import brickpi3
import grovepi

import movement
import mag_sense

def stop(BP):
        movement.stop(BP)

def selfCal(BP,port_f,port_b,bst = 1):
        try:
            calib = [999,-1,999,-1]
            #      f-b  f-w  b-b  b-w

            #sweep right for .7 sec tracking the mac and min sensor readings
            t = .7
            while t > 0:
                movement.setSpeed(BP,2 * bst,-2 * bst,1)     #sweeping speed (base at 2cm/s with multiplier)
                sensor_value_f = grovepi.analogRead(port_f)
                sensor_value_b = grovepi.analogRead(port_b)
                print("front: "+str(sensor_value_f)+" | back: "+str(sensor_value_b))  #reads in and prints sensors

                if 250 > sensor_value_f > calib[1]:    #if (front) sensor value is greater than current highest, reassign 
                        calib[1] = sensor_value_f
                elif sensor_value_f < calib[0]:        #if (front) sensor value is lower than current highest, reassign 
                        calib[0] = sensor_value_f

                if 250 > sensor_value_b > calib[3]:    #if (back) sensor value is greater than current highest, reassign 
                        calib[3] = sensor_value_b
                elif sensor_value_b < calib[2]:        #if (back) sensor value is lower than current highest, reassign
                        calib[2] = sensor_value_b
                time.sleep(.01)
                t -= .01                 #decrement t to "keep track" of time

            #same process as above except in opposite direction (left)
            t = 1.2
            while t > 0:
                movement.setSpeed(BP,-2 * bst,2 * bst,1)
                sensor_value_f = grovepi.analogRead(port_f)
                sensor_value_b = grovepi.analogRead(port_b)
                print("front: "+str(sensor_value_f)+" | back: "+str(sensor_value_b))

                if 250 > sensor_value_f > calib[1]:
                        calib[1] = sensor_value_f
                elif sensor_value_f < calib[0]:
                        calib[0] = sensor_value_f

                if 250 > sensor_value_b > calib[3]:
                        calib[3] = sensor_value_b
                elif sensor_value_b < calib[2]:
                        calib[2] = sensor_value_b
                time.sleep(.01)
                t -= .01


            eq = (calib[0] + calib[1]) / 2      #calculate eq of front from lowest and highest calced above ^^

            #travel back going right, when a sensor value within 20% of eq is found, stop and return calib
            while not(sensor_value_f < calib[0] + .25 * (eq - calib[0])):
                movement.setSpeed(BP,1 * bst,-1 * bst,1)
                sensor_value_f = grovepi.analogRead(port_f)
                print(sensor_value_f)
                
                time.sleep(.005)

            
            movement.setSpeed(BP,0,0)    
            
            return calib
        except Exception as error: 
            print("selfCal: "+str(error))

        except KeyboardInterrupt:
            stop(BP)   
       
def lightSensorCal(BP,port):
        # SIG,NC,VCC,GND
        grovepi.pinMode(port,"INPUT")
        try:
            tot = 0
            times = 0
            while times < 50:
                # Get sensor value
                sensor_value = grovepi.analogRead(port)
                print(sensor_value)
                if sensor_value < 900:
                        tot += sensor_value 
                        times += 1
                time.sleep(.03)
            
            return tot/(times)
        except Exception as error: 
            print("lightSensorCal: "+str(error))

        except KeyboardInterrupt:
            stop(BP)   # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.

def lineCal(BP,port_b,port_f):
    calib = [-1,-1,-1,-1]
    for x in range(0,4):
        if 0 <= x <= 1:
            calib[x] = lightSensorCal(BP,port_b)
        else:
            calib[x] = lightSensorCal(BP,port_f)

        t = 5
        while t >= 0:
            print(str(t) + " seconds until next calibration")
            time.sleep(1)
            t -= 1 

    return calib

def findRightEdge(BP,port,eq,full_b,full_w):
    t = 1
    while t > 0:
        movement.setSpeed(BP,1.5,-1.5,1)
        sensor_value_f = grovepi.analogRead(port)
        print("front: "+str(sensor_value_f))
        time.sleep(.01)
        t -= .01

    while not(sensor_value_f < full_b + .2 * (eq - full_b)):
        movement.setSpeed(BP,-1,1,1)
        sensor_value_f = grovepi.analogRead(port)
        print(sensor_value_f)
        time.sleep(.005)
    
    movement.setSpeed(BP,0,0)

def findLeftEdge(BP,port,eq,full_b,full_w):
    t = 1
    while t > 0:
        movement.setSpeed(BP,-1.5,1.5,1)
        sensor_value_f = grovepi.analogRead(port)
        print("front: "+str(sensor_value_f))
        time.sleep(.01)
        t -= .01

    while not(sensor_value_f < full_b + .2 * (eq - full_b)):
        movement.setSpeed(BP,1,-1,1)
        sensor_value_f = grovepi.analogRead(port)
        print(sensor_value_f)
        time.sleep(.005)
    
    movement.setSpeed(BP,0,0)

def sweep(BP,port,eq,full_b,full_w,tm,direc = 1):
    '''does an initial sweep right/left(1/-1) and then the opposite way to find the line'''
    t = tm * .75
    sensor_value_f = grovepi.analogRead(port)
    while not(sensor_value_f < full_b + .2 * (eq - full_b)) and t > 0:
        movement.setSpeed(BP,direc * 1.5,direc * -1.5,1)
        sensor_value_f = grovepi.analogRead(port)
        print("front: "+str(sensor_value_f))
        time.sleep(.01)
        t -= .01

    t = tm * 2
    while not(sensor_value_f < full_b + .2 * (eq - full_b)) and t > 0:
        movement.setSpeed(BP,direc * -1.5,direc * 1.5,1)
        sensor_value_f = grovepi.analogRead(port)
        print(sensor_value_f)
        time.sleep(.01)
        t -= .01
    
    movement.setSpeed(BP,0,0)

def sweepForw(BP,port,eq,full_b,full_w,tm = 5,direc = 1):
    t = tm
    sensor_value_f = grovepi.analogRead(port)
    while not(sensor_value_f < full_b + .2 * (eq - full_b)) and t > 0:
        movement.setSpeed(BP,direc * 1.5,direc * 1.5,1)
        sensor_value_f = grovepi.analogRead(port)
        print("front: "+str(sensor_value_f))
        time.sleep(.01)
        t -= .01

    movement.burst(BP,3,5,.5)
    movement.burst(BP,1,-2,-2)
    sweep(BP,port,eq,full_b,full_w,2)
    movement.setSpeed(BP,0,0)

def turnAround(BP,port,eq,full_b,full_w):
    movement.burst(BP,2,1,8)
    sensor_value_f = grovepi.analogRead(port)
    while not(sensor_value_f < full_b + .2 * (eq - full_b)):
        movement.setSpeed(BP,-2,2,1)
        sensor_value_f = grovepi.analogRead(port)
        print(sensor_value_f)
        time.sleep(.005)
        
    movement.setSpeed(BP,0,0)

def followOneL(BP,speed,port,full_b,full_w,drc,t = 0,bst = 0):
    try:
        launch = input("Hit enter for launch")
        print(launch)
        
        grovepi.pinMode(port,"INPUT")
        eq = (full_b + full_w)/2

        #left edge proportional line following
        p = 1*((1/.5) / (eq - full_b))  #more black = turn left
        #   ^-k value multiplier
        #         ^-expon gain constant/coefficient --> determines threshold where speed multiplier = 1

        speed_a = speed 
        if t > 0:
            while t > 0:
                start_time = time.time()
                sensor_read = grovepi.analogRead(port) 
                print("sensor: "+str(sensor_read))

                if(sensor_read < full_w * 1.5):
                    if sensor_read < eq:
                        motor_mod = speed * (p * (eq - sensor_read)) ** 1.2
                    else:
                        motor_mod = - speed * (p * (sensor_read- eq)) ** 1.2

                    if (eq - .15 * (eq - full_b)) > sensor_read:
                        movement.setSpeed(BP,(speed - motor_mod),(speed + motor_mod),drc)  #turn left (speed_l v | speed_r ^)
                    elif (eq + .15 * (full_w - eq)) < sensor_read:
                        movement.setSpeed(BP,(speed - motor_mod),(speed + motor_mod),drc)  #turn right (speed_l ^ | speed_r v)
                    else:
                        movement.setSpeed(BP,speed,speed,drc)

                time.sleep(.001)
                t -= time.time() - start_time
        elif bst > 0:
            while True:
                speed_a = speed * bst
                sensor_read = grovepi.analogRead(port) 
                print("sensor: "+str(sensor_read))

                if(sensor_read < full_w * 1.5):
                    if sensor_read < eq:
                        motor_mod = speed * (p * (eq - sensor_read)) ** 1.2
                    else:
                        motor_mod = - speed * (p * (sensor_read- eq)) ** 1.2

                    #      turn left (speed_l v | speed_r ^)             turn right (speed_l ^ | speed_r v)
                    if ((eq - .15 * (eq - full_b)) > sensor_read) or ((eq + .15 * (full_w - eq)) < sensor_read):
                        movement.setSpeed(BP,(speed - motor_mod),(speed + motor_mod),drc) 
                    #      turn left (speed_l v | speed_r ^)             turn right (speed_l ^ | speed_r v)
                    elif((eq - .15 * (eq - full_b)) < sensor_read < eq) or (eq < sensor_read < eq + .15 * (full_w - eq)):
                        movement.setSpeed(BP,(speed_a - motor_mod),(speed_a + motor_mod),drc)
                    else:
                        movement.setSpeed(BP,speed,speed,drc)

                time.sleep(.001)
        else:
            while True:
                sensor_read = grovepi.analogRead(port) 
                print("sensor: "+str(sensor_read))

                if(sensor_read < full_w * 1.5):
                    if sensor_read < eq:
                        motor_mod = speed * (p * (eq - sensor_read)) ** 1.2
                    else:
                        motor_mod = - speed * (p * (sensor_read- eq)) ** 1.2

                    if (eq - .15 * (eq - full_b)) > sensor_read:
                        movement.setSpeed(BP,(speed - motor_mod),(speed + motor_mod),drc)  #turn left (speed_l v | speed_r ^)
                    elif (eq + .15 * (full_w - eq)) < sensor_read:
                        movement.setSpeed(BP,(speed - motor_mod),(speed + motor_mod),drc)  #turn right (speed_l ^ | speed_r v)
                    else:
                        movement.setSpeed(BP,speed,speed,drc)

                time.sleep(.001)
                    
        movement.setSpeed(BP,0,0)
    except Exception as error: 
        print("followOneL:",error)
    except KeyboardInterrupt:
        stop(BP)   

def followOneR(BP,speed,port,full_b,full_w,drc,t = 0,bst = 0):
    try:
        launch = input("Hit enter for launch")
        print(launch)
        
        grovepi.pinMode(port,"INPUT")
        eq = (full_b + full_w)/2

        #left edge proportional line following
        p = 1*((1/.5) / (eq - full_b))  #more black = turn left
        #   ^-k value multiplier
        #       ^-expon gain constant/coefficient --> determines threshold where speed multiplier = 1
        
        if t > 0:
            while t > 0:
                start_time = time.time()
                sensor_read = grovepi.analogRead(port) 
                print("sensor: "+str(sensor_read))

                if(sensor_read < full_w * 1.5):
                    if sensor_read < eq:
                        motor_mod = speed * (p * (eq - sensor_read)) ** 1.2
                    else:
                        motor_mod = - speed * (p * (sensor_read- eq)) ** 1.2

                    if (eq - .15 * (eq - full_b)) > sensor_read:
                        movement.setSpeed(BP,(speed + motor_mod),(speed - motor_mod),drc)  #turn right (speed_l ^ | speed_r v)
                    elif (eq + .15 * (full_w - eq)) < sensor_read:
                        movement.setSpeed(BP,(speed + motor_mod),(speed - motor_mod),drc)  #turn left (speed_l v | speed_r ^)
                    else:
                        movement.setSpeed(BP,speed,speed,drc)

                time.sleep(.001)
                t -= time.time() - start_time
        elif bst > 0:
            while True:
                speed_a = speed * bst
                sensor_read = grovepi.analogRead(port) 
                print("sensor: "+str(sensor_read))

                if(sensor_read < full_w * 1.5):
                    if sensor_read < eq:
                        motor_mod = speed * (p * (eq - sensor_read)) ** 1.2
                    else:
                        motor_mod = - speed * (p * (sensor_read- eq)) ** 1.2

                    #      turn left (speed_l v | speed_r ^)             turn right (speed_l ^ | speed_r v)
                    if ((eq - .15 * (eq - full_b)) > sensor_read) or ((eq + .15 * (full_w - eq)) < sensor_read):
                        movement.setSpeed(BP,(speed + motor_mod),(speed - motor_mod),drc) 
                    #      turn left (speed_l v | speed_r ^)             turn right (speed_l ^ | speed_r v)
                    elif((eq - .15 * (eq - full_b)) < sensor_read < eq) or (eq < sensor_read < eq + .15 * (full_w - eq)):
                        movement.setSpeed(BP,(speed_a + motor_mod),(speed_a - motor_mod),drc)
                    else:
                        movement.setSpeed(BP,speed,speed,drc)

                time.sleep(.001)
        else:
            while True:
                sensor_read = grovepi.analogRead(port) 
                print("sensor: "+str(sensor_read))

                if(sensor_read < full_w * 1.5):
                    if sensor_read < eq:
                        motor_mod = speed * (p * (eq - sensor_read)) ** 1.2
                    else:
                        motor_mod = - speed * (p * (sensor_read- eq)) ** 1.2

                    if (eq - .15 * (eq - full_b)) > sensor_read:
                        movement.setSpeed(BP,(speed + motor_mod),(speed - motor_mod),drc)  #turn right (speed_l ^ | speed_r v)
                    elif (eq + .15 * (full_w - eq)) < sensor_read:
                        movement.setSpeed(BP,(speed + motor_mod),(speed - motor_mod),drc)  #turn left (speed_l v | speed_r ^)
                    else:
                        movement.setSpeed(BP,speed,speed,drc)

                time.sleep(.001)

    except Exception as error: 
        print("followOneR:",error)
    except KeyboardInterrupt:
        stop(BP) 

def followOneMagL(BP,speed,port,full_b,full_w,detect = [],drc = 1):
    try:
        
        eq = (full_b + full_w)/2
        esc = 0

        #left edge proportional line following
        p = 1*((1/.5) / (eq - full_b))  #more black = turn left
        #   ^-k value multiplier
        #       ^-expon gain constant/coefficient --> determines threshold where speed multiplier = 1
        
        while (esc == 0):
            sensor_read = grovepi.analogRead(port)
            magnet = mag_sense.analogSensing(BP,2000,2100) 
            print("sensor: "+str(sensor_read))

            if(magnet == 1):
                detect[0] = 1
                esc = 1
                speed *= .25

            if(sensor_read < full_w * 1.5):
                if sensor_read < eq:
                    motor_mod = speed * (p * (eq - sensor_read)) ** 1.2
                else:
                    motor_mod = - speed * (p * (sensor_read- eq)) ** 1.2

                if (eq - .15 * (eq - full_b)) > sensor_read:
                    movement.setSpeed(BP,(speed - motor_mod),(speed + motor_mod),drc)  #turn left (speed_l v | speed_r ^)
                elif (eq + .15 * (full_w - eq)) < sensor_read:
                    movement.setSpeed(BP,(speed - motor_mod),(speed + motor_mod),drc)  #turn right (speed_l ^ | speed_r v)
                else:
                    movement.setSpeed(BP,speed,speed,drc)

            time.sleep(.001)  

    except Exception as error: 
        print("followOneMagL:",error)
    except KeyboardInterrupt:
        stop(BP) 

def followOneMagR(BP,speed,port,full_b,full_w,detect = [],drc = 1):
    try:
        eq = (full_b + full_w)/2

        #left edge proportional line following
        p = 1*((1/.5) / (eq - full_b))  #more black = turn left
        #   ^-k value multiplier
        #       ^-expon gain constant/coefficient --> determines threshold where speed multiplier = 1
        
        while True:
            sensor_read = grovepi.analogRead(port) 
            print("sensor: "+str(sensor_read))

            if(sensor_read < full_w * 1.5):
                if sensor_read < eq:
                    motor_mod = speed * (p * (eq - sensor_read)) ** 1.2
                else:
                    motor_mod = - speed * (p * (sensor_read- eq)) ** 1.2

                if (eq - .15 * (eq - full_b)) > sensor_read:
                    movement.setSpeed(BP,(speed + motor_mod),(speed - motor_mod),drc)  #turn right (speed_l ^ | speed_r v)
                elif (eq + .15 * (full_w - eq)) < sensor_read:
                    movement.setSpeed(BP,(speed + motor_mod),(speed - motor_mod),drc)  #turn left (speed_l v | speed_r ^)
                else:
                    movement.setSpeed(BP,speed,speed,drc)

            time.sleep(.001)

    except Exception as error: 
        print("followOneMagR:",error)
    except KeyboardInterrupt:
        stop(BP) 

def magFollow(BP,speed,port,full_b,full_w):
    try:
        launch = input("Hit enter for launch")
        print(launch)

        grovepi.pinMode(port,"INPUT")

        # In this logic, 1 indicates currently detected and 0 indicates condition not detected
        detect = [0,0]
        #         ^--Indicator magnet (Site A - 1 | Site B - 2 | Site C - 3)
        #           ^--Drop off magnet

        while True:
            #based on values of detect choose and specific edge and sensor (front or back)
            if detect[0] == 1:
                movement.burst(BP,1.5,3,1)
                followOneMagR(BP,speed,port,full_b,full_w,detect)
            else:
                followOneMagL(BP,speed,port,full_b,full_w,detect)

            time.sleep(.01)

    except Exception as error: 
        print("magFollow:",error)
    except KeyboardInterrupt:
        stop(BP)

'''
RUNNING CONTROL MODEL
'''
def followL(BP,speed,port,full_b,full_w,eq,ang,ang_off,detect,bst = 1,snsr = 0):
    try:                
        drc = 1
        esc = 0

        #calculate the exponential gain constant 
        p = (1/.5) / (eq - full_b)            
        #         ^-- gain threshhold --> determines threshold where speed multiplier = 1
        
        speed_a = speed * bst    #boosted speed (used under certain error conditions)
        i = 0                    #counter that controls frequency of sensor reading

        while (esc != 1):
            sensor_read = grovepi.analogRead(port)                             #read light sensor value
            print("Left edge - sensor("+ str(snsr) +"): "+ str(sensor_read))

            #sensor readings (every other iteration)
            if (i % 2 == 0):
                detect[4] += mag_sense.analogSensing(BP,1940,2160)     #magnet 1 detection  
                detect[6] = int(movement.gyroVal(BP) - ang_off > ang)  #obstacle detection
                if detect[6] == 0 and detect[7] == 1:                  #obstacle sweep detection
                    detect[7] = 2             

            #break out of loop if one sensor detects new control condition
            if (detect[snsr] + detect[4] + detect[6] + detect[7] >= 1):
                movement.setSpeed(BP,0,0)
                esc = 1

            #if not abnormal sensor reading (filter)
            elif(sensor_read < full_w * 1.5):
                #calculate the change in motor speed for each based on error, gain, and exponent
                if sensor_read < eq:
                    motor_mod = speed * (p * (eq - sensor_read)) ** 1.2
                else:
                    motor_mod = - speed * (p * (sensor_read - eq)) ** 1.2

                # lost the line     
                if(eq + .90 * (full_w - eq) < sensor_read) and detect[7] == 0 and detect[10] != 2:
                    movement.setSpeed(BP,0,0)
                    detect[snsr] = 1
                # line detected
                else:
                    if detect[10] == 1:
                        detect[10] == 2

                    #      turn left (speed_l v | speed_r ^)             turn right (speed_l ^ | speed_r v)
                    if ((eq - .15 * (eq - full_b)) > sensor_read) or ((eq + .15 * (full_w - eq)) < sensor_read):
                        movement.setSpeed(BP,(speed - motor_mod),(speed + motor_mod),drc) 
                    #      turn left (speed_l v | speed_r ^)             turn right (speed_l ^ | speed_r v)
                    elif((eq - .05 * (eq - full_b)) < sensor_read < eq) or (eq < sensor_read < eq + .05 * (full_w - eq)):
                        movement.setSpeed(BP,(speed_a * 2 - motor_mod),(speed_a * 2 + motor_mod),drc)
                    #      turn left (speed_l v | speed_r ^)             turn right (speed_l ^ | speed_r v)
                    elif((eq - .15 * (eq - full_b)) < sensor_read < eq) or (eq < sensor_read < eq + .15 * (full_w - eq)):
                        movement.setSpeed(BP,(speed_a - motor_mod),(speed_a + motor_mod),drc)

            i += 1
            time.sleep(.001)

    except Exception as error: 
        print("followL:",error)
    except KeyboardInterrupt:
        stop(BP)


def followR(BP,speed,port,full_b,full_w,eq,ang,ang_off,detect,bst = 1,snsr = 0):
    try:
        drc = 1
        esc = 0

        #calculate the exponential gain constant 
        p = (1/.5) / (eq - full_b)            
        #         ^-- gain threshhold --> determines threshold where speed multiplier = 1
        speed_a = speed * bst
        i = 0

        while (esc != 1):
            sensor_read = grovepi.analogRead(port) 
            print("Right edge - sensor("+ str(snsr) +"): "+ str(sensor_read))

            if (i % 2 == 0):
                detect[5] = mag_sense.analogSensing(BP,1940,2160)

                detect[6] = int(movement.gyroVal(BP) - ang_off > ang) 
                if detect[8] == 1: 
                    detect[4] = 4 * mag_sense.analogSensing(BP,1950,2150)
                if detect[6] == 0 and detect[7] == 1:
                    detect[7] = 2 

            if (detect[snsr] + detect[5] + detect[6] + detect[7] >= 1):
                movement.setSpeed(BP,0,0)
                esc = 1

            elif(sensor_read < full_w * 1.5):
                if sensor_read < eq:
                    motor_mod = speed * (p * (eq - sensor_read)) ** 1.2
                else:
                    motor_mod = - speed * (p * (sensor_read - eq)) ** 1.2

                #      lost the line     
                if(eq + .90 * (full_w - eq) < sensor_read) and detect[7] == 0  and detect[10] != 2:
                    movement.setSpeed(BP,0,0)
                    detect[snsr] = 1
                else:
                    if detect[10] == 1:
                        detect[10] == 2

                    #      turn left (speed_l v | speed_r ^)             turn right (speed_l ^ | speed_r v)
                    if ((eq - .15 * (eq - full_b)) > sensor_read) or ((eq + .15 * (full_w - eq)) < sensor_read):
                        movement.setSpeed(BP,(speed + motor_mod),(speed - motor_mod),drc) 
                    #      turn left (speed_l v | speed_r ^)             turn right (speed_l ^ | speed_r v)
                    elif((eq - .05 * (eq - full_b)) < sensor_read < eq) or (eq < sensor_read < eq + .05 * (full_w - eq)):
                        movement.setSpeed(BP,(speed_a * 2 + motor_mod),(speed_a * 2 - motor_mod),drc)
                    #      turn left (speed_l v | speed_r ^)             turn right (speed_l ^ | speed_r v)
                    elif((eq - .15 * (eq - full_b)) < sensor_read < eq) or (eq < sensor_read < eq + .15 * (full_w - eq)):
                        movement.setSpeed(BP,(speed_a + motor_mod),(speed_a - motor_mod),drc)
                    #      lost the line     
                    elif(eq + .95 * (full_w - eq) < sensor_read) and detect[7] == 0:
                        movement.setSpeed(BP,0,0)
                        detect[snsr] = 1

            i += 1
            time.sleep(.001)

    except Exception as error: 
        print("followR:",error)
    except KeyboardInterrupt:
        stop(BP)
