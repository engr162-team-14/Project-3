import time     
import brickpi3  
import grovepi  
import numpy as np

import csv

from movement import setSpeed
from movement import turnPi
from movement import speedControl
from movement import pt2Pt
from movement import pt2PtScan
from movement import stop

from sensors import lightCalib
from sensors import lightTest
from sensors import lightVal
from sensors import imuCalib


def calibrate():
    lightCalib()
    calib = imuCalib()
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

    return imu_calib

def scanArea(BP,speed,imu_calib):
    img_data = np.full((25,25),10)

    for r in range(25):
        if r % 2 == 0:
            img_data[r] = pt2PtScan(BP,speed,(0,r),(25,r),imu_calib)
            pt2Pt(BP,speed,(25,r),(25,r + 1),imu_calib)
        else:
            img_data[r] = pt2PtScan(BP,speed,(25,r),(0,r),imu_calib)
            pt2Pt(BP,speed,(0,r),(0,r + 1),imu_calib)

    print("Scan complete")
    return img_data
    

def writeData(img_data):
    with open('DC3_Data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(img_data)

    print("Image data written to DC3_Data.csv")

if __name__ == '__main__':
    BP = brickpi3.BrickPi3()
    imu_calib = calibrate()
    img_data = scanArea(BP,5,imu_calib)
    writeData(img_data)
        