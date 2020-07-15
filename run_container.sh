#!/bin/bash
sudo docker build . -t ros_controller

# Networking settings
ROS_MASTER_URI="http://192.168.0.181:11311"
ROS_IP="192.168.0.134"
sudo docker run \
	--hostname controller \
	-it \
	--rm \
	--net=host \
	-e ROS_MASTER_URI=$ROS_MASTER_URI \
    -e ROS_IP=$ROS_IP \
	--add-host controller:127.0.0.1 \
	--add-host coppeliaSim:192.168.0.134 \
	--add-host ros:192.168.0.181 \
	ros_controller:latest \
#  	bash
#	--user root \
