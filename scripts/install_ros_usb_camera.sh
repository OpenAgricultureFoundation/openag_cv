#/usr/bin/bash

# Getting ROS sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu jessie main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

# Update APT
sudo apt-get update
sudo apt-get -y upgrade

#Install python tools
sudo apt-get -y install python-pip python-setuptools python-yaml python-distribute python-docutils python-dateutil python-six
sudo pip install rosdep rosinstall_generator wstool rosinstall

# Resolve ROS dependencies
sudo rosdep init
rosdep update

# Make ROS sources workspace
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws

# Install ROS_COMM distribution
rosinstall_generator ros_comm usb_cam pid --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-ros_comm-wet.rosinstall
wstool init src indigo-ros_comm-wet.rosinstall

# Install external dependencies
sudo apt-get -y install libconsole-bridge-dev liblz4-dev

# Build and install packages into final directory
cd ~/ros_catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:jessie
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo -j2

# Source shell interpreter
source /opt/ros/indigo/setup.bash
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

# I am done with this ...
echo "DONE"
