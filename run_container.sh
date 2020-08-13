#!/bin/bash
sudo docker build . -t ros_controller

# Networking settings
ROS_MASTER_URI="http://10.10.10.10:11311"
ROS_IP="10.10.10.101"
sudo docker run \
	--hostname controller \
	-it \
	--rm \
	--net=host \
	-e ROS_MASTER_URI=$ROS_MASTER_URI \
    	-e ROS_IP=$ROS_IP \
	--add-host controller:127.0.0.1 \
	--add-host coppeliaSim:10.10.10.101 \
	--add-host niryo-desktop:10.10.10.10 \
	ros_controller:latest \
#  	bash
#	--user root \
