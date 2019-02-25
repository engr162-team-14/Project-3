import time      #import time library
import brickpi3  #import BrickPi3 drivers
import grovepi   #import the GrovePi drivers
import time

import movement        #import other modules
import line_follow
import mag_sense

def calibrate(BP):
    '''calibrate each of the necessary sensors'''
    calib = line_follow.selfCal(BP,1,0,1)         #calibrate light sensors
    #calib = [0, 24, 15, 47]
    
    calib.append(movement.gyroCalib(BP)[0])        #calibrate Gyro
    mag_sense.analogHallCalib(BP)                  #calibrate Hall Sensor
    movement.gate(BP,0)

    print(calib)
    return calib

def obstacleTest(BP,ang,ang_off):
    '''move a certain speed over a certain distance with gyro speed boost'''
    BP.set_motor_position(BP.PORT_A, 0)
    movement.speedControlGyro(BP,10,150,ang,ang_off)

    BP.set_motor_position(BP.PORT_A, 110)
    time.sleep(2)

    movement.speedControlGyro(BP,25,5,ang,ang_off)
    movement.speedControlGyro(BP,-25,30,ang,ang_off)
    BP.set_motor_position(BP.PORT_A, 0)
 
def lineCalibrate(BP):
    '''calibrate and find full_w, and full_b for both light sensors'''
    calib = line_follow.selfCal(BP,1,0)
    print(calib)
    #calib = [0, 17, 3, 33]
    return calib

def magnetLineTest(BP,calib):
    '''test magnet sensing and line following'''
    line_follow.magFollow(BP,4,0,calib[2],calib[3])

def switchLineFollowTest(BP,calib):
    line_follow.followOneL(BP,6,1,calib[0],calib[1],1,3)
    movement.burst(BP,1.75,3,1)
    line_follow.followOneR(BP,6,1,calib[0],calib[1],1)

    #line_follow.followOneL(BP,6,1,calib[0],calib[1],1,5)
    #line_follow.findRightEdge(BP,1,eq,calib[0],calib[1])
    #line_follow.followOneR(BP,6,1,calib[0],calib[1],1)

def doubleControlTest(BP):
    pass
    #      f      b    --> [full_b_f,full_w_f,full_b_b,full_w_b]
    
    '''line following -- forward'''
    #line_follow.doubleControl(BP,5,1,calib[2],calib[3],0,calib[0],calib[1],'l',1)


    '''line following -- back'''
    #line_follow.doubleControl(BP,5,0,calib[0],calib[1],1,calib[2],calib[3],'l',-1)
    
    # movement.follow(BP,10,0,1,0,0,0)
    # line_follow.followR(BP,5,0,2,0,0)
    #                              R     
    #                           eq b w

def controlC(BP,speed,port_f,port_b,calib,ang,ang_off,def_edge = 'l',def_snsr = 1,direc = 1):
    '''autonomous control algorithm for clockwise travel'''
    try:
        drop_site = int(input("Enter Drop Site Number (1-3): "))

        grovepi.pinMode(port_f,"INPUT")
        grovepi.pinMode(port_b,"INPUT")

        # In this logic, 1 indicates currently detected and 0 indicates condition not detected
        detect = [0,0,0,0,0,0,0,0,0,0,2]
                #0^--white detected on LEFT by FRONT sensor (snsr = 0)
                #1  ^--white detected on LEFT by BACK sensor (snsr = 1)
                #2    ^--white detected on RIGHT by FRONT sensor (snsr = 2)
                #3      ^--white detected on RIGHT by BACK sensor (snsr = 3)
                #4        ^--magnet 1 detected (1-3)
                #5          ^--magnet 2 detected
                #6            ^-- angle > 'ang' detected
                #7              ^--up the hill? (0 - no hill | 1 - on hill | 2 - over hill)
                #8                ^--object dropped off (1 - yes | 0 - no)
                #9                  ^--magnet passed on way back (1 - yes | 0 - no)
                #10                  ^-- gap passed (2 - yes | 1 - in process | 0 - no) 

        eq_f = (calib[0] + calib[1])/2      #calculate equilibrium values
        eq_b = (calib[2] + calib[3])/2

        edge = 'r'                          #default edge for start of course (to avoid broken lines)

        while True:
            if (detect[8] == 0):           #if the object if not dropped off, speed boost amount
                speed_c = speed * 1.25
            else:
                speed_c = speed

            if(detect[0] + detect[1] == 2):        
                detect[0] = 0
                movement.burst(BP,.75,3,3)
                line_follow.sweep(BP,1,eq_f,calib[0],calib[1],2,1)
                detect[10] = 1

            if(detect[4] == drop_site):
                movement.burst(BP,3.5,-5,-5)
                movement.burst(BP,2.5,5,1)
                line_follow.sweepForw(BP,1,eq_f,calib[0],calib[1])
                edge = 'r'
                detect[4] = -1

            elif (detect[4] > 3):
                movement.burst(BP,1.5,3,3)
                line_follow.turnAround(BP,1,eq_f,calib[0],calib[1])
                edge = 'r'
                detect[10] = 0
                detect[4] = 0
                detect[8] += 1

            elif (detect[4] > 1 and detect[8] > 1):
                line_follow.turnAround(BP,1,eq_f,calib[0],calib[1])

            elif(detect[5] == 1):
                movement.dump(BP,0,110,ang,ang_off)
                line_follow.turnAround(BP,1,eq_f,calib[0],calib[1])
                edge = 'r'

                detect[4] = 0
                detect[8] = 1

            elif(detect[6] == 1):
                #movement.burst(BP,3,-4,-4)
                #line_follow.sweep(BP,1,eq_f,calib[0],calib[1],2,1)
                #movement.burst(BP,3,5,5)
                obst = movement.obstBoost(BP,ang,ang_off)
                while(obst[1] > 0):
                     movement.setSpeed(BP,speed * (obst[0] * .95),speed * obst[0])
                     obst[1] -= 1
                     time.sleep(.025)
                     
                detect[7] = 1          
                movement.setSpeed(BP,0,0)


            elif(detect[7] == 2):
                movement.setSpeed(BP,0,0)
                line_follow.sweep(BP,1,eq_f,calib[0],calib[1],.75,-1)
                detect[7] = 0

            
            if edge == 'l':
                if detect[0] == 0:
                    line_follow.followL(BP,speed_c,1,calib[0],calib[1],eq_f,ang,ang_off,detect,1.25,0)
                else:
                    line_follow.followL(BP,speed_c,0,calib[2],calib[3],eq_b,ang,ang_off,detect,1.25,1)
            else:
                if detect[0] == 0:
                    line_follow.followR(BP,speed_c,1,calib[0],calib[1],eq_f,ang,ang_off,detect,1.25,2)
                else:
                    line_follow.followR(BP,speed_c,0,calib[2],calib[3],eq_b,ang,ang_off,detect,1.25,3)

            speed_c = speed

            time.sleep(.005)
    
    except Exception as error: 
        print("controlC: ",error)
    except KeyboardInterrupt:
        movement.stop(BP)

def controlCC(BP,speed,port_f,port_b,calib,ang,ang_off,def_edge = 'l',def_snsr = 1,direc = 1):
    '''autonomous control algorithm for counter-clockwise travel'''
    try:
        drop_site = int(input("Enter Drop Site Number (1-3): "))

        grovepi.pinMode(port_f,"INPUT")
        grovepi.pinMode(port_b,"INPUT")

        # In this logic, 1 indicates currently detected and 0 indicates condition not detected
        detect = [0,0,0,0,0,0,0,0,0,0,0]
                #0^--white detected on LEFT by FRONT sensor (snsr = 0)
                #1  ^--white detected on LEFT by BACK sensor (snsr = 1)
                #2    ^--white detected on RIGHT by FRONT sensor (snsr = 2)
                #3      ^--white detected on RIGHT by BACK sensor (snsr = 3)
                #4        ^--magnet 1 detected (1-3)
                #5          ^--magnet 2 detected
                #6            ^-- angle > 'ang' detected
                #7              ^--up the hill? (0 - no hill | 1 - on hill | 2 - over hill)
                #8                ^--object dropped off (1 - yes | 0 - no)
                #9                  ^--magnet passed on way back (1 - yes | 0 - no)
                #10                  ^-- gap passed (2 - yes | 1 - in process | 0 - no) 

        eq_f = (calib[0] + calib[1])/2
        eq_b = (calib[2] + calib[3])/2

        edge = 'l'

        while True:
            if (detect[8] == 0):
                speed_c = speed * 1.25
            else:
                speed_c = speed

            if(detect[0] + detect[1] == 2):
                detect[0] = 0
                movement.burst(BP,.75,3,3)
                line_follow.sweep(BP,1,eq_f,calib[0],calib[1],2,1)
                detect[10] = 1

            if(detect[4] == drop_site):
                movement.burst(BP,3.5,-5,-5)
                movement.burst(BP,2.5,5,1)
                line_follow.sweepForw(BP,1,eq_f,calib[0],calib[1])
                edge = 'r'
                detect[4] = -1  

            elif (detect[4] > 3):
                movement.burst(BP,1.5,3,3)
                line_follow.turnAround(BP,1,eq_f,calib[0],calib[1])
                edge = 'r'
                detect[10] == 0
            
            elif (detect[4] > 1 and detect[8] > 1):
                line_follow.turnAround(BP,1,eq_f,calib[0],calib[1])

            elif(detect[5] == 1):
                movement.dump(BP,0,110,ang,ang_off)
                line_follow.turnAround(BP,1,eq_f,calib[0],calib[1])
                edge = 'r'

                detect[4] = 0
                detect[8] = 1

            elif(detect[6] == 1):
                obst = movement.obstBoost(BP,ang,ang_off)
                while(obst[1] > 0):
                     movement.setSpeed(BP,speed * (obst[0] * .95),speed * obst[0])
                     obst[1] -= 1
                     time.sleep(.025)
                     
                detect[7] = 1           
                movement.setSpeed(BP,0,0)

            elif(detect[7] == 2):
                movement.setSpeed(BP,0,0)
                line_follow.sweep(BP,1,eq_f,calib[0],calib[1],.75,-1)
                detect[7] = 0

            
            if edge == 'l':
                if detect[0] == 0:
                    line_follow.followL(BP,speed_c,1,calib[0],calib[1],eq_f,ang,ang_off,detect,1.25,0)
                else:
                    line_follow.followL(BP,speed_c,0,calib[2],calib[3],eq_b,ang,ang_off,detect,1.25,1)
            else:
                if detect[0] == 0:
                    line_follow.followR(BP,speed_c,1,calib[0],calib[1],eq_f,ang,ang_off,detect,1.25,2)
                else:
                    line_follow.followR(BP,speed_c,0,calib[2],calib[3],eq_b,ang,ang_off,detect,1.25,3)

            speed_c = speed

            time.sleep(.005)
    
    except Exception as error: 
        print("controlCC: ",error)
    except KeyboardInterrupt:
        movement.stop(BP)
   
def speedTrack(BP,speed,dist):
    '''speed track algorithm'''
    try:
        launch = int(input("Enter go for launch"))
        print(launch)

        movement.speedControl(BP,speed,dist)       
    
    except Exception as error: 
        print("speedTrack: ",error)
    except KeyboardInterrupt:
        movement.stop(BP)


if __name__ == '__main__':
    BP = brickpi3.BrickPi3()  #creating a BrickPi object of the brickpi3 class
    #BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
    calib = calibrate(BP)
    ang_off = calib[4]
    ang = 3

    print("-----------------------------------------\n-  1. Standard Course - Counter Clockwise"+
    " \n-  2. Standard Course - Clockwise\n-  3. Speed Course")
    choice = int(input("Enter Choice: "))

    if choice == 1:
        controlCC(BP,4,1,0,calib,ang,ang_off)
    elif choice == 2:
        controlC(BP,4,1,0,calib,ang,ang_off)
    else:
        speedTrack(BP,20,325)

