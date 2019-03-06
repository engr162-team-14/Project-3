from MPU9250 import MPU9250
import sys
import smbus
import time
from math import sqrt
import brickpi3

mpu9250 = MPU9250()
feele = open("magDist.csv","w")

BP = brickpi3.BrickPi3()
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.TOUCH)

gap=int(input("Enter the distance between steps in cm: "))
steps=int(input("Enter the number of steps: "))
          
tFLAG=0
edgeNew=0
edgeOld=0

feele.write("Distance,")
feele.write("X axis,")
feele.write("Y axis,")
feele.write("Z axis,")
feele.write("Magnitude,")
feele.write("Time,\n")

time.sleep(0.1)

t0=time.time()
t_init=t0
try:
    for x in range(0,steps):

        print("\nMove the sensor to be "+str((x+1)*gap)+" cm away from the magnet")
        print("Press the button when you are ready to measure.")
        
        while not tFLAG:
            try:
                edgeNew = BP.get_sensor(BP.PORT_1)
                if edgeOld==1 and edgeNew==0:
                    tFLAG=1
                edgeOld=edgeNew
            except brickpi3.SensorError as error:
                print(error)
        tFLAG=0
        mag = mpu9250.readMagnet()
        print("X axis: " + str(mag['x']) + "\tY axis: " +str(mag['y']) + "\tZ axis: " + str(mag['z']) + "\tTime: " +str(time.time()-t_init))
        time.sleep(0.1)
        if(mag['x']!= 0 or mag['y']!= 0 or mag['z']!= 0):
            feele.write(str((x+1)*gap))
            feele.write(" cm,")
            feele.write(str(mag['x']))
            feele.write (",")
            feele.write(str(mag['y']))
            feele.write (",")
            feele.write(str(mag['z']))
            feele.write(",")
            feele.write(str(sqrt(mag['x']*mag['x']+mag['y']*mag['y']+mag['z']*mag['z'])))
            feele.write(",")

            t=time.time()
            delt=t-t0
            feele.write(str(delt))
            feele.write(",\n")
            
    feele.close()
    
except KeyboardInterrupt:
    feele.close()
    sys.exit()
    
