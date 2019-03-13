'''
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