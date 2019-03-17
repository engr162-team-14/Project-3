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