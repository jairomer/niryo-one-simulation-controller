#!/bin/bash
sudo docker build . -t ros_controller

sudo docker run \
	-it \
	--rm \
	--net=host \
	-e DISPLAY=$DISPLAY \
	-e ROS_MASTER_URI=$ROS_MASTER_URI \
	ros_controller:latest \
#	bash
  	