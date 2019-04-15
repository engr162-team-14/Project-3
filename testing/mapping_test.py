from mapping import Map, Dir, State
import sensors

import time     
import brickpi3  
import grovepi  

def calibrate(BP):
    #gyro = sensors.gyroCalib(BP)
    sensors.gyroCalib(BP)
    sensors.leftUltraCalib(BP)
    sensors.irCalib()

    calib = sensors.imuCalib()
    imu_calib = {
        "mpu": calib[0],
        "accel": calib[1],
        "gyro": calib[2],
        "mag": calib[3],
        "flter": calib[4],
        "biases": calib[5],
        "dly": calib[6],
        "std": calib[7]
    }
    # Map = Map(...)
    # BP = brickpi3.BrickPi3()
    # Cal = Calibration(BP,imu_calib,gyro)
    # return Cal

    return imu_calib

if __name__ == '__main__':
    BP = brickpi3.BrickPi3()
    imu_calib = calibrate(BP)

    origin = input("Enter origin coordinates separted by space: ").split()
    origin = [int(i) for i in origin]
    map_num = input("Enter map number: ")


    map = Map(origin,int(map_num),direc=Dir.UP)
    print(map.grid)
    map._addPoint((1,1),State.EXPL)
    print(map.grid)
    map._addPoint((2,1),State.EXPL)
    print(map.grid)
    map.pushInfo()