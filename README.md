# AVS
ROS packages for controlling Autonomous Vehicle System

**cmd_msgs** - ROS package contains special message to control Autonomous Vehicle. It needs to be on both sides: master (host-computer) and robot (Jetson TK-1). 

**master_station** - ROS package for controlling Autonomous Vehicle. It reads keyboard events and sends corresponding commands. It needs to be launched on master (host-computer) with root privilegies. Settings are in /cfg/master_station_param.yaml.

**move_controller** - ROS package that controls Autonomous Vehicle. It gets commands from /cmd_vel topic and sends them to drivers. It needs to be launched on robot (Jetson TK-1) with root privilegies (need access to I2C bus). Settings are in /cfg/move_controller_param.yaml.
The package uses JHPWMPCA9685 library for connection with PCA9685 board.


