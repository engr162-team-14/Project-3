import time #import time library
import brickpi3 #import BrickPi3 drivers
import grovepi #import the GrovePi drivers
import math

import movement

def stop(BP):
    movement.stop(BP)

def analogHallCalib(BP):
    BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.CUSTOM, [(BP.SENSOR_CUSTOM.PIN1_ADC)]) # Configure for an analog on sensor port pin 1, and poll the analog line on pin 1.
    print("calibrating analogHall...")
    while True:
        try:
            print(BP.get_sensor(BP.PORT_4)[0])
            print("success")
            break
        except Exception:
            pass

        time.sleep(.05)
    
def analogHall(BP):
    try:
        value = BP.get_sensor(BP.PORT_4)[0] # read the sensor port value
        print("Hall Sensor:",value,"\n")
        return value

    except Exception as error: 
        print("analogHall: "+str(error))
    except KeyboardInterrupt:
        stop(BP)

def analogHallTest(BP):
    BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.CUSTOM, [(BP.SENSOR_CUSTOM.PIN1_ADC)]) # Configure for an analog on sensor port pin 1, and poll the analog line on pin 1.

    try:
        while True:
            # read the sensor value
            # BP.get_sensor retrieves a sensor value.
            # BP.PORT_1 specifies that we are looking for the value of sensor port 1.
            # BP.get_sensor returns a list of 4 values.
            #     The first is the pin 1 analog line value (what we want to display).
            #     The second is the pin 6 analog line value.
            #     The third is the pin 5 digital value.
            #     The fourth is the pin 6 digital value.
            try:
                value = BP.get_sensor(BP.PORT_4)[0] # read the sensor port value
                print(value)
                #print("Raw value: %4d   Voltage: %5.3fv" % (value, (value / (4095.0 / BP.get_voltage_5v())))) # print the raw value, and calculate and print the voltage as well
            except brickpi3.SensorError as error:
                print(error)
            
            time.sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.

    except Exception as error: 
        print("analogHall: "+str(error))
    except KeyboardInterrupt:
        stop(BP)

def digitalHallTest(BP,port):       
    grovepi.pinMode(port,"INPUT")
    try:
            value = grovepi.digitalRead(port)
            return value
        
    except Exception as error: 
            print("digitalHall: "+str(error))
    except KeyboardInterrupt:
        stop(BP)

def digitalHall(BP,port):       
    grovepi.pinMode(port,"INPUT")
    try:
        while True:
            value = grovepi.digitalRead(port)
            print(value)
            time.sleep(0.1)
        
    except Exception as error: 
            print("digitalHallTest: "+str(error))
    except KeyboardInterrupt:
        stop(BP)

def analogSensing(BP,sns_low,sns_high):
    reading = analogHall(BP)
    if reading > sns_high or reading < sns_low:
        return 1
    else:
        return 0

def findEffectiveRange(BP,sns_low,sns_high):
    try:
        while True:
            mag = analogSensing(BP,sns_low,sns_high)
            if (mag == 1):
                movement.setSpeed(BP,-3,-3)
            else:
                movement.setSpeed(BP,0,0)
    except Exception as error: 
        print(error)
    except KeyboardInterrupt:
        movement.stop(BP)