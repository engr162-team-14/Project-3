import time     
import brickpi3  
import grovepi 
import movement


#functional tests
def calibrate(): 
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



if __name__ == '__main__':
    BP = brickpi3.BrickPi3()
    imu_calib = calibrate()
    movement.imuFiltered(imu_calib)
