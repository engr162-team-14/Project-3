#!/usr/bin/env python
#
# GrovePi Example for using the analog read command to read analog sensor values
#
# The GrovePi connects the Raspberry Pi and Grove sensors.  You can learn more about GrovePi here:  http://www.dexterindustries.com/GrovePi
#
# Have a question about this example?  Ask on the forums here:  http://forum.dexterindustries.com/c/grovepi
#
'''
## License

The MIT License (MIT)

GrovePi for the Raspberry Pi: an open source platform for connecting Grove Sensors to the Raspberry Pi.
Copyright (C) 2017  Dexter Industries

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
'''

from MPU9250 import MPU9250
import time
import math
import brickpi3 # import the BrickPi3 drivers

mpu9250 = MPU9250()

BP = brickpi3.BrickPi3()
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.TOUCH)

tFLAG=0
edgeNew=0
edgeOld=0
fid = open("FieldMagData.csv","w")
fid2 = open("FieldVecData.csv","w")
fid.write("Points,")
fid2.write("Points,")

xwidth=int(input("Enter the total number of x steps: "))
ywidth=int(input("Enter the total number of y steps: "))

for q in range(0,xwidth+1):
    fid.write(str(q))
    fid.write(",")
    fid2.write(str(q))
    fid2.write(",")
    fid2.write(",")
    fid2.write(",")
    
fid.write("\n")
fid2.write("\n")

time.sleep(0.1)

try:
    for x in range(0,xwidth):
        fid.write(str(x))
        fid.write(",")
        fid2.write(str(x))
        fid2.write(",")
        for y in range(0,ywidth):
            print("Move to location ("+str(x)+" , "+str(y)+") and press the button")
            while not tFLAG:
                try:
                    edgeNew = BP.get_sensor(BP.PORT_1)
                    if edgeOld==1 and edgeNew==0:
                        tFLAG=1
                    edgeOld=edgeNew
                except brickpi3.SensorError as error:
                    print(error)
            try:
                mag = mpu9250.readMagnet()
                total_mag=math.sqrt(mag['x']*mag['x']+mag['y']*mag['y']+mag['z']*mag['z'])
                fid.write(str(total_mag))
                fid.write(",")
                fid2.write(str(mag['x']))
                fid2.write(",")
                fid2.write(str(mag['y']))
                fid2.write(",")
                fid2.write(str(mag['z']))
                fid2.write(",")
            except IOError:
                print ("IO Error")
            tFLAG=0
        fid.write("\n")
        fid2.write("\n")
    fid.close()
    fid2.close()
    print("Data Collection Completed.\n")


except KeyboardInterrupt:
    BP.reset_all()
    fid.close()
    fid2.close()
