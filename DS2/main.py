import time     
import brickpi3  
import grovepi 
import movement


#functional tests
def calibrate(BP):
    movement.gyroCalib(BP)


if __name__ == '__main__':
    BP = brickpi3.BrickPi3()
    calibrate(BP)
