# Simulation Controller

A controller for the Niryo One simulation on coppeliaSIM using the ROS interface. 

## Compile it
clone in catkin_ws/src. 

`catkin_make`

## Run it

0. Prepare for current setup. 
 - Configuration for the environment is contained in the docker run command on `run_container.sh`
 - All relevant hosts on the ROS network should be added as hosts. 
 - Set the ROS master uri to the one on your setup.
 - Set the ROS ip to the one of the interface connecting to the ROS network.

1. Execute script to setup ROS environment as a docker container and start node.
    `./run_container`

