''' MAPPING EXTRA GRAVEYARD

    def evalJunction(self,has_front_path,has_left_path,has_right_path):
        self._setPoint(self.cur_loc)
        turn_ang = None

        # Approaching new junction
        if self.cur_x == len(self.grid[0]) or self.cur_y == 0 or \
           self._getPoint((self.cur_x + 1, self.cur_y)) != State.UNEXPL and self._getPoint((self.cur_x, self.cur_y + 1)) != State.UNEXPL \
           and self._getPoint((self.cur_x - 1, self.cur_y)) != State.UNEXPL and self._getPoint((self.cur_x, self.cur_y - 1)) != State.UNEXPL: 
            if has_front_path:
                self._setPointRelative(Dir.UP, State.UNEXPL)
            if has_left_path:
                self._setPointRelative(Dir.LEFT, State.UNEXPL)
            if has_right_path:
                self._setPointRelative(Dir.RIGHT, State.UNEXPL)

        # Evaluate junction
        if self._getPoint((self.cur_x + 1, self.cur_y)) == State.UNEXPL:
            turn_ang = 90
        elif self._getPoint((self.cur_x, self.cur_y)) == State.UNEXPL:
            turn_ang = 90

        return turn_ang

'''

'''
def pt_2_pt_abs (BP, imu_calib, speed, pt_1, pt_2, init_ang, length_conv = 5, haz_mode = Hazard.NO_HAZARDS):
    try:
        distance = length_conv * getDistance(pt_1[0], pt_1[1], pt_2[0], pt_2[1])
        angle = init_ang - getAngle(pt_1[0], pt_1[1], pt_2[0], pt_2[1])
        turnPi(BP, angle)
        if haz_mode == Hazard.NO_HAZARDS:
            speedControl(BP, imu_calib, speed, distance, haz_mode = haz_mode)
        else:
            pos = speedControl(BP, imu_calib,speed,distance, haz_mode = haz_mode)
            # calculate new route and get there...
        turnPi(BP, -angle)
    except Exception as error: 
        print("pt_2_pt_abs",error)
    except KeyboardInterrupt:
        stop(BP)

Isaac = gyroVal(BP)

def go_to_90 (BP):
    try:
        current = gyroVal(BP)
        turnDegree = 90 + Isaac - current
        return turnDegree
    except Exception as error: 
        print("go_to_90",error)
    except KeyboardInterrupt:
        stop(BP)
def go_to_180 (BP):
    try:
        current = gyroVal(BP)
        turnDegree = 180 + Isaac - current
        return turnDegree
    except Exception as error: 
        print("go_to_180",error)
    except KeyboardInterrupt:
        stop(BP)
def go_to_90_2 (BP):
    try:
        current = gyroVal(BP)
        turnDegree = -90 + Isaac - current
        return turnDegree
    except Exception as error: 
        print("go_to_90_2",error)
    except KeyboardInterrupt:
        stop(BP)
def go_to_0 (BP):
    try:
        current = gyroVal(BP)
        turnDegree = Isaac - current
        return turnDegree
    except Exception as error: 
        print("go_to_0",error)
    except KeyboardInterrupt:
        stop(BP)

def pt_2_pt2 (BP,imu_calib, x1, x2, y1, y2):
    #funtion navegates the robot from one point (x1, y1) to another point (x2, y2) using the linear components
    try:
        veci = x2 - x1
        vecj = y2 - y1
        if(veci < 0):
            deg = go_to_90(BP)
            turnPi(BP, deg)
            speedControl(BP, imu_calib, 6, abs(veci))
            if(vecj < 0):
                deg = go_to_180(BP)
                turnPi(BP, deg)
                speedControl(BP, imu_calib, 6, abs(vecj))
            else:
                deg = go_to_0(BP)
                turnPi(BP, 0)
                speedControl(BP, imu_calib, 6, abs(vecj))
        else:
            deg = go_to_90_2(BP)
            turnPi(BP, deg)
            speedControl(BP, imu_calib, 6, abs(veci))
            if(vecj < 0):
                go_to_180
                turnPi(BP, deg)
                speedControl(BP, imu_calib, 6, abs(vecj))
            else:
                deg = go_to_0(BP)
                turnPi(BP, deg)
                speedControl(BP, imu_calib, 6, abs(vecj))
    except Exception as error: 
        print("pt_2_pt2",error)
    except KeyboardInterrupt:
        stop(BP)
            

'''       

'''
right parallel to wall


if sensor == Sensor.RIGHT:
            cur_dist = getUltras(BP)[2]
            while cur_ang <= (init_ang + dtheta):
                if cur_dist < min_dist:
                    min_dist = cur_dist
                    targ_angle = cur_ang
                setSpeed(BP,sweep_spd,-sweep_spd)

                cur_ang = gyroVal(BP)
                cur_dist = getUltras(BP)[2]
                time.sleep(dt)
            setSpeed(BP,0,0)
            
            while cur_ang >= (init_ang - dtheta):
                if cur_dist < min_dist:
                    min_dist = cur_dist
                    targ_angle = cur_ang
                setSpeed(BP,-sweep_spd,sweep_spd)

                cur_ang = gyroVal(BP)
                cur_dist = getUltras(BP)[2]
                time.sleep(dt)
            setSpeed(BP,0,0)
'''


'''
checking for gyro off
abs(cur_angle - sensors.gyroVal(BP)) > 4
'''


'''
gyro guiding once reached set dist

                    gyro_error = -1
                    gyro_error_p = 0
                    gyro_integ = 0
                    gyro_dt = .1

                    dps = (speed * (360/(7* pi)))

                    # while ultras[0] > set_dists[0] and ultras[1] < set_dists[1] + bfr_dist and ultras[2] < set_dists[2] + bfr_dist:
                    while True:
                        gyro_error = cur_angle - sensors.gyroVal(BP)                         #error = system (gyro) dev from desired state (target_deg)
                        gyro_integ = gyro_integ + (gyro_dt * (gyro_error + gyro_error_p)/2)  #integral feedback (trapez approx)
                        gyro_output = gyro_kp * (gyro_error) + gyro_ki * (gyro_integ)        #PI feedback response
                        gyro_error_p = gyro_error
                        
                        BP.set_motor_dps(BP.PORT_C, dps + gyro_output)   
                        BP.set_motor_dps(BP.PORT_B, dps - gyro_output) 
                        act_dist = np.multiply(sensors.getUltras(BP), cos(radians(sensors.gyroVal(BP) - cur_angle)))

                        if abs(act_dist[1] - set_dists[1]) > 1.5 and sensor == Sensor.LEFT:
                            break
 
                        time.sleep(gyro_dt)
                    '''



'''
mazNav pt 1 (wall guiding in corridors)
            # sweep to parallel with wall
            turn_ang = parallelToWall(BP,cur_angle,sweep_spd = 1.5, sensor = sensor)
            print("turn_ang:",turn_ang)

            turnPi(BP, turn_ang)            

            
            # while ultras[0] > set_dists[0] and ultras[1] < set_dists[1] + bfr_dist and ultras[2] < set_dists[2] + bfr_dist:
            while True:
                errors = np.subtract(set_dists, act_dist)
                integs = np.add(integs, (np.multiply(dt, np.divide(np.add(errors, errors_p), 2))))
                outputs  = np.add(np.multiply(kp, errors), np.multiply(ki, integs))
                print(outputs)
                errors_p = errors

                if sensor == Sensor.RIGHT:
                    #apprch right wall --> (+) error --> add (+) error to right w (^ spd) & subtr (+) error to left w (v spd) 
                    #apprch left wall --> (-) error --> add (-) error to right w (v spd) & subtr (-) error to left w (^ spd)
                    setSpeed(BP, speed - outputs[2], speed + outputs[2])  
                elif sensor == Sensor.LEFT:
                    #apprch right wall --> (-) error --> subtr (-) error to right w (^ spd) & add (-) error to left w (v spd) 
                    #apprch left wall --> negative error --> subtr (+) error to right w (v spd) & add (+) error to left w (^ spd)
                    setSpeed(BP, speed + outputs[1], speed - outputs[1]) 

                act_dist = np.multiply(sensors.getUltras(BP), cos(radians(sensors.gyroVal(BP) - cur_angle)))

                time.sleep(1)
'''

'''
Time of each step -- mazeNav (wall guiding)
                init_time = time.time()
                errors = np.subtract(set_dists, act_dist)
                print("errors = np.sub - time: %d", time.time() - init_time)
                
                init_time = time.time()
                print("ultras:",act_dist)
                print("print(ultras) - time: %d", time.time() - init_time)

                init_time = time.time()
                integs = np.add(integs, (np.multiply(dt, np.divide(np.add(errors, errors_p), 2))))
                print("integs... - time: %d", time.time() - init_time)

                init_time = time.time()
                outputs  = np.add(np.multiply(kp, errors), np.multiply(ki, integs))
                print("outputs... - time: %d", time.time() - init_time)

                init_time = time.time()
                errors_p = errors
                print("errors_p - time: %d", time.time() - init_time)

                init_time = time.time()
                if sensor == Sensor.RIGHT:
                    #apprch right wall --> (+) error --> add (+) error to right w (^ spd) & subtr (+) error to left w (v spd) 
                    #apprch left wall --> (-) error --> add (-) error to right w (v spd) & subtr (-) error to left w (^ spd)
                    setSpeed(BP, speed - outputs[2], speed + outputs[2])  
                elif sensor == Sensor.LEFT:
                    #apprch right wall --> (-) error --> subtr (-) error to right w (^ spd) & add (-) error to left w (v spd) 
                    #apprch left wall --> negative error --> subtr (+) error to right w (v spd) & add (+) error to left w (^ spd)
                    setSpeed(BP, speed + outputs[1], speed - outputs[1]) 
                print("left or right - time: %d", time.time() - init_time)

                init_time = time.time()
                act_dist = np.multiply(sensors.getUltras(BP), cos(radians(sensors.gyroVal(BP) - cur_angle)))
                print("act_dist = ... - time: %d", time.time() - init_time)

                init_time = time.time()
                time.sleep(dt)
                print("sleep - time: %d", time.time() - init_time)
'''








'''
Things to integrate into code

--PID
kp = .3
ki = .7
kd = .1

error = set - curr
integ = integ + (dt * error * .5)                    #triangular approx
    or 
integ = integ + (dt * (error + error_p)/2)           #trapezoidal approx (better)
deriv = (error - error_p)/dt

output  = (kp * error) + (ki * integral)  + (kd * deriv)
error_p = error
'''