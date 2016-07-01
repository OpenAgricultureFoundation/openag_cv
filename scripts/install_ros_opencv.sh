#/usr/bin/bash

BASEDIR=`pwd`

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
rosinstall_generator ros_comm usb_cam cv_bridge image_transport --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-ros_comm-wet.rosinstall
wstool init src indigo-ros_comm-wet.rosinstall

# Install external dependencies
sudo apt-get -y install libconsole-bridge-dev liblz4-dev libtheora-bin libtheora-dev

# Install ffmpeg for mjpeg compression
cd /usr/local/src
sudo git clone https://github.com/mirror/x264
sudo chown -R pi x264
cd x264
./configure --host=arm-unknown-linux-gnueabi --enable-static --enable-shared --disable-opencl --enable-pic
make
sudo make install

cd /usr/local/src
sudo git clone https://github.com/FFmpeg/FFmpeg.git
sudo chown -R pi FFmpeg
cd FFmpeg
./configure --arch=armel --target-os=linux --enable-gpl --enable-libtheora --enable-libx264 --enable-nonfree --enable-pic
make
sudo make install

# Adding swap space
sudo dd if=/dev/zero of=/swapfile bs=1M count=1024
sudo mkswap /swapfile
sudo swapon /swapfile

# Build and install dependencies
cd ~/ros_catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:jessie

# Move modified usb_cam_node
cp $BASEDIR/../src/usb_cam/usb_cam.cpp ~/ros_catkin_ws/src/usb_cam/src/

# Compile and install final version
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo -j2

# Delete swap file
sudo swapoff /swapfile
sudo rm /swapfile

# Source shell interpreter
source /opt/ros/indigo/setup.bash
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

# I am done with this ...
echo "DONE"
