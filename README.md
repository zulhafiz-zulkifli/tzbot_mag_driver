# tzbot_mag_driver  [![License: MIT](https://img.shields.io/github/license/zulhafiz-zulkifli/tzbot_mag_driver?style=flat-square)](https://github.com/zulhafiz-zulkifli/tzbot_mag_driver/blob/ros2/LICENSE)


**Author**: Zulhafiz Zulkifli

**Description**: ROS driver for TZBOT AGV magnetic sensor using serial communication.

## Overview

This repository offers a ROS package and an API written in python to communicate with the TZBOT AGV magnetic sensor using serial communication. This package support RS-485 and RS-232 in broadcast mode only. This package does not support RS-485 and RS-232 in poll mode or CAN interface. Currently only tested on TZS-MAG-1600-B model. However, the code can be easily modified to work with other models.

## Dependencies
- [serial](https://github.com/pyserial/pyserial)
```shell
sudo apt install python3-serial
```

## Installation

1.  Clone the repository
```shell
cd ros2_ws/src/
git clone -b ros2 https://github.com/zulhafiz-zulkifli/tzbot_mag_driver.git
```

2. Build the package
```shell
cd ros2_ws
colcon build
source install/setup.bash
```

3. Launch the driver
```shell
ros2 launch tzbot_mag main.launch.py
```


## ROS API

### Published Topics

***`~deviation_right` (std_msgs/Float32)***   
Deviation from the center of magnetic sensor to right fork in mm.

***`~deviation_middle` (std_msgs/Float32)***   
Deviation from the center of magnetic sensor to middle fork in mm.

***`~deviation_left` (std_msgs/Float32)***   
Deviation from the center of magnetic sensor to left fork in mm.

***`~cell_state` (std_msgs/String)***   
Magnetic sensor cell's status which depends on the configured sensitivity. For example, a magnetic sensor with 15 cells would have 15 bits to express their states.


### Parameters 
All ROS parameters can be changed inside config/params.yaml before launching the driver.

***`~communication_interface` (string, default: "rs485")***   
Serial communication interface. Only support "rs485" and "rs232"

***`~serial_port` (string, default: "/dev/ttyS0" )***   
Serial port name where the sensor is connected.

***`~baudrate` (int, default: 9600)***   
 Rate at which the information is shared to the communication channel. Only support 9600, 19200, 38400 and 115200 bps.

***`~frequency` (float, default: 10.0)***    
Frequency at which serial data from the sensor is read and ROS topics are published in Hz.