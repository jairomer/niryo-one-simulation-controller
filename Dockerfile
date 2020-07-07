# ROS kinetic docker deployment to run ROS on Ubuntu16_04.
FROM ubuntu:16.04
MAINTAINER jairomer@protonmail.com

COPY hosts /etc/hosts

WORKDIR /root
RUN useradd -ms /bin/bash controller

# Setup ROS dependencies. 
RUN \
  apt-get update && \
  apt-get install -y software-properties-common && \
  sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
  apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install system dependencies. 
RUN \
  sed -i 's/# \(.*multiverse$\)/\1/g' /etc/apt/sources.list && \
  add-apt-repository universe && \
  add-apt-repository multiverse && \
  apt-get update && \
  apt-get -y upgrade && \
  apt-get install -y build-essential sudo -qqy x11-apps && \
  apt-get install -y software-properties-common && \
  apt-get install -y byobu curl git htop man unzip vim wget && \
  apt-get install -y ros-kinetic-desktop-full && \
  apt-get install -y python-rosinstall python-rosinstall-generator && \
  apt-get install -y python-wstool build-essential python3-pip xauth && \
  apt-get install -y qt5-default ros-kinetic-robot-state-publisher \
    ros-kinetic-moveit ros-kinetic-rosbridge-suite ros-kinetic-joy \
    ros-kinetic-ros-control ros-kinetic-ros-controllers \
    ros-kinetic-tf2-web-republisher &&\
  pip3 install jsonpickle && \
  rm -rf /var/lib/apt/lists/*

RUN pip3 install -U catkin_tools

RUN rosdep init

USER controller
WORKDIR /home/controller

RUN rosdep update && mkdir -p catkin_ws/src && cd catkin_ws && catkin init && cd src && \
    git clone https://github.com/NiryoRobotics/niryo_one_ros.git . && \
    mkdir simulation_controller && mkdir simulation_controller/src && \
    mkdir simulation_controller/launch && cd ..

COPY CMakeLists.txt catkin_ws/src/simulation_controller/ 
COPY package.xml catkin_ws/src/simulation_controller/ 
COPY src/* catkin_ws/src/simulation_controller/src/
COPY launch/* catkin_ws/src/simulation_controller/launch/
RUN cd catkin_ws && bash -c 'source /opt/ros/kinetic/setup.bash && catkin_make'

#CMD bash 
CMD bash -c "source /opt/ros/kinetic/setup.bash && source catkin_ws/devel/setup.bash\
      && roslaunch simulation_controller simulation_controller_node.launch"  
