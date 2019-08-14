from mapping import Map, Dir, State
import sensors

import time     
import brickpi3  
import grovepi  

from miniunit import lineno, mu_check, mu_run

def calibrate(BP):
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

    return imu_calib

def test__setPoint():
    line_fail_arr = [-1]

    # origin = input("Enter origin coordinates separted by space: ").split()
    # origin = [int(i) for i in origin]
    # map_num = input("Enter map number: ")
    origin = (0,0)
    map_num = 1

    map = Map(origin,int(map_num),direc=Dir.UP)
    map._setPoint((1,1), State.EXPL)
    map._setPoint((2,1), State.EXPL)
    map._setPoint((2,2), State.EXPL)
    map._setPoint((3,3), State.EXPL)

    # mainly visual testing, more point validation in test__get_point()
    mu_check(map.grid[0][3] == State.EXPL, line_fail_arr, lineno())
    mu_check(len(map.grid) == 4, line_fail_arr, lineno())
    
    map.pushInfo()

    return line_fail_arr[0]

def test__getPoint():
    line_fail_arr = [-1]

    origin = (0,0)
    map_num = 2

    map = Map(origin,int(map_num),direc=Dir.UP)
    map._setPoint((1,1), State.EXPL)
    map._setPoint((2,1), State.EXPL)
    map._setPoint((2,2), State.EXPL)
    map._setPoint((3,3), State.EXPL)
    
    mu_check(map._getPoint((0,0)) == State.ORIG, line_fail_arr, lineno())
    mu_check(map._getPoint((1,1)) == State.EXPL, line_fail_arr, lineno())
    mu_check(map._getPoint((1,0)) == State.UNKWN, line_fail_arr, lineno())
    mu_check(map._getPoint((3,3)) == State.EXPL, line_fail_arr, lineno())
    mu_check(map._getPoint((1,0)) == State.UNKWN, line_fail_arr, lineno())

    mu_check(len(map.grid) == 4, line_fail_arr, lineno())

    return line_fail_arr[0]

def test_updateLocation():
    line_fail_arr = [-1]

    origin = (0,0)
    map_num = 3

    map = Map(origin,int(map_num),direc=Dir.UP)
    map.updateLocation()
    map.cur_direc = Dir.RIGHT
    map.updateLocation()
    map.updateLocation()
    map.cur_direc = Dir.UP
    map.updateLocation()
    map.updateLocation()
    map.cur_direc = Dir.LEFT
    map.updateLocation()
    map.updateLocation()
    map.cur_direc = Dir.DOWN
    map.updateLocation()

    mu_check(map._getPoint((0,0)) == State.ORIG, line_fail_arr, lineno())
    mu_check(map._getPoint((1,0)) == State.UNKWN, line_fail_arr, lineno())
    mu_check(map._getPoint((0,1)) == State.EXPL, line_fail_arr, lineno())
    mu_check(map._getPoint((2,1)) == State.EXPL, line_fail_arr, lineno())
    mu_check(map._getPoint((2,2)) == State.EXPL, line_fail_arr, lineno())
    mu_check(map._getPoint((1,3)) == State.EXPL, line_fail_arr, lineno())
    mu_check(map._getPoint((0,3)) == State.EXPL, line_fail_arr, lineno())
    mu_check(map._getPoint((0,2)) == State.CUR, line_fail_arr, lineno())

    map.pushInfo()

    return line_fail_arr[0]
    

def test_addHazard():
    line_fail_arr = [-1]

    origin = (0,0)
    map_num = 4

    map = Map(origin,int(map_num),direc=Dir.UP)
    map.updateLocation()
    map.addHazard(State.HEAT,200)
    map.cur_direc = Dir.RIGHT
    map.updateLocation()
    map.updateLocation()
    map.addHazard(State.MAG,70)
    map.cur_direc = Dir.UP
    map.updateLocation()
    map.cur_direc = Dir.RIGHT
    map.updateLocation()

    mu_check(map._getPoint((0,0)) == State.ORIG, line_fail_arr, lineno())
    mu_check(map._getPoint((0,1)) == State.EXPL, line_fail_arr, lineno())
    mu_check(map._getPoint((0,2)) == State.HEAT, line_fail_arr, lineno())
    mu_check(map._getPoint((1,1)) == State.EXPL, line_fail_arr, lineno())
    mu_check(map._getPoint((2,1)) == State.EXPL, line_fail_arr, lineno())
    mu_check(map._getPoint((3,1)) == State.MAG, line_fail_arr, lineno())
    mu_check(map._getPoint((2,2)) == State.EXPL, line_fail_arr, lineno())
    mu_check(map._getPoint((3,2)) == State.CUR, line_fail_arr, lineno())

    mu_check(map.hazard_info[1][0] == "Fire", line_fail_arr, lineno())
    mu_check(map.hazard_info[1][1] == "Temperature (C)", line_fail_arr, lineno())
    mu_check(map.hazard_info[1][2] == 200, line_fail_arr, lineno())
    mu_check(map.hazard_info[1][3] == 0, line_fail_arr, lineno())
    mu_check(map.hazard_info[1][4] == 2, line_fail_arr, lineno())

    map.pushInfo()


    return line_fail_arr[0]

if __name__ == '__main__':
    BP = brickpi3.BrickPi3()
    
    mu_run(test__setPoint)
    mu_run(test__getPoint)
    mu_run(test_updateLocation)
    mu_run(test_addHazard)