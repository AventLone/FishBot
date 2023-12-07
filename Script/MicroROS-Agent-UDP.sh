#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/avent/Public/MyProjects/FishBot/Micro-ROS-Agent/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888 -v6
