# tzbot_mag_driver  [![License: MIT](https://img.shields.io/github/license/zulhafiz-zulkifli/tzbot_mag_driver?style=flat-square)](https://github.com/zulhafiz-zulkifli/tzbot_mag_driver/blob/ros2/LICENSE)


**Author**: Zulhafiz Zulkifli

**Description**: ROS driver for TZBOT AGV magnetic sensor using serial communication.

## Overview

This repository offers a ROS package and an API written in python to communicate with the TZBOT AGV magnetic sensor using serial communication. This package support RS-485 and RS-232 in broadcast mode only. This package does not support RS-485 and RS-232 in poll mode or CAN interface. Currently only tested on TZS-MAG-1600-B model. However, the code can be easily modified to work with other models.

## Dependencies:
1. [serial](https://github.com/pyserial/pyserial)
    - `sudo apt install python3-serial`

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

3. Launch the package
```shell
ros2 launch tzbot_mag main.launch.py
```

Parameters for the driver change be changed inside config/params.yaml.

| Parameter | Default value | Description |
| ------ | ------ | ------ |
| communication_interface | "rs485" | Serial communication interface. |
| serial_port | "/dev/ttyS0" | 	Serial port name where the sensor is connected. |
| baudrate | 9600 |  Rate at which the information is shared to the communication channel. Only support 9600, 19200, 38400 or 115200 bps. |
| frequency | 10.0 | Frequency at which serial data from the sensor is read and ROS topics are published in Hz.|
