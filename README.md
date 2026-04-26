# ORA IMU MicroROS System
In the robots developement for the International Ground Vehicle Competition (IGVC), a system needed to be developed for interfacing with lower level peripherals and communicating with devices upstream.

## System Components to Implement
1. Custom PCB that implements the following features:
   1. ESP32 on board for running MicroROS framework
   2. Use of external 9-DOF IMU
   3. BJT circuit for controlling 24V stack light
   
2. Software running on ESP32 to implement the following features:
   1. Data capturing of external IMU
   2. Publish IMU data to ROS topic
   3. Service messages controlling robot stack light

