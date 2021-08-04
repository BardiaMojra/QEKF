#!/bin/bash

_project_name='QVPose'

sudo apt update

sudo apt install python-pip -y
sudo pip install numpy
sudo apt install libsuitesparse-dev -y
sudo apt install libeigen3-dev -y
#pip install cvxopt
sudo apt install cvxopt -y
sudo apt install libarmadillo-dev -y
sudo apt install libeigen3-dev -y
sudo apt install python-cvxopt -y

sudo apt install ros-melodic-image-transport ros-melodic-camera-info-manager -y
sudo apt install ros-melodic-tf ros-melodic-tf2 ros-melodic-eigen-conversions -y
sudo apt install ros-melodic-tf2-geometry-msgs ros-melodic-geometry-msgs -y
sudo apt install libcurl4-openssl-dev -y


# must install
sudo apt install ros-melodic-geographic-msgs

# install python modules
pip install filterpy

exit 0

# setup, compile and install openCV -- running scripts directly wont work.
sudo bash ~/QVPose/opencv_setup/opencv_setup.sh # run commands in terminal - wont work as a script
sudo bash ~/QVPose/build_opencv.sh # run commands in terminal - wont work as a script


source /opt/ros/melodic/setup.bash

## setup catkin workspace -----> no need to init catkin ws every time
mkdir -p ~/QVPose/catkin_ws/src # run commands in terminal - wont work as a script
cd ~/QVPose/catkin_ws/ # run commands in terminal - wont work as a script
catkin_make


# EOF
