#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/avent/Public/MyProjects/FishBot/Micro-ROS-Agent/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyUSB0 -v
