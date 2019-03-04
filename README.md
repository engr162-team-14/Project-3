Project-3

Agenda
    - Mapping
        - Map and grid square class structure
        - Major functions:
            - creation and logic
                - junction/ choice logic
                - tracking current position
                - creating map as going
                    - path vs. hazards
            - transmitting map
                - go through example code and understand
                - functions that allow us to transmit when we want and choices on what we transmit????
                - integrate this stuff into current system
        - Integrating Map stuff with maze_nav

    - Sensor Stuff
        - IMU Magnetometer
            - IMU initial testing to get data back
            - Running through filters successfully
            - Grabbing magnetometer data at will
        - IR Sensor
            - Go through example code and understand
        - Ultrasonic
        - Integrating Sensor data/ filtering into maz_nav

    - Testing
        - Test functions in main.py for each POC task

    - Documention ****ALL*****
        - Make a good effort to try to comment things (especially potentially confusing stuff)
        - Make notes in design notebook for rationale on choices (e.g. why use pi vs pid, why window vs kalman filter, etc.)

    

Questions
    1. For "no enter zone" on hazards, what constitutes entering zone? Paperclip loc on robot or any part?
    2. If prev Q answer is "any part" --> WHeat happens if the IMU does not have enough range on its magnetometer
