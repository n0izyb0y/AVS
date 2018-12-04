# AVS
ROS packages for controlling Autonomous Vehicle System

**cmd_msgs** - ROS package contains special message to control Autonomous Vehicle. Need to be on both sides: master (host-computer) and robot (Jetson TK-1). 

**master_station** - ROS package for control Autonomous Vehicle. Read keyboard events and send corresponding commands. Need to be launch on master (host-computer) with root privilegies. Settings are in /cfg/master_station_param.yaml.

**move_controller** - ROS package that controls Autonomous Vehicle. Get commands from /cmd_vel topic and send it to drivers. Need to be launched on robot (Jetson TK-1) with root privilegies (need access to I2C bus). Settings are in /cfg/move_controller_param.yaml.
Uses JHPWMPCA9685 library for connecting with servo drivers.


