import time     
import brickpi3  
import grovepi  
import movement

from MPU9250 import MPU9250


#functional tests
def calibrate(BP):
    movement.gyroCalib(BP)

    calib = movement.imuCalib()
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


def IMUTest(BP):
    try:
        mpu9250 = MPU9250()
        while True:
            gyro = mpu9250.readGyro()
            print(gyro['x'])

    except Exception as error: 
        print("IMUTest:",error)
    except KeyboardInterrupt:
        movement.stop(BP)



if __name__ == '__main__':
    BP = brickpi3.BrickPi3()
    imu_calib = calibrate(BP)