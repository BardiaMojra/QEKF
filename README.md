# Requirements & Setup

This repository was developed on
Ubuntu 18.04
ROS Melodic
OpenCV 3.2.0 with contrib

## ROS Melodic

ROS Melodic [installation](http://wiki.ros.org/melodic/Installation/Ubuntu)

## OpenCV Setup

Run the following commands in the terminal.

```bash
cd opencv_setup && sudo bash opencv_setup.sh # clone and add submodules to git
sudo bash build_opencv.sh # build and install opencv with contrib
```

## Environment Setup

Run the following commands in the terminal.

```bash
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

# install python modules
pip install filterpy
```

## Raspberry Pi Installation Note

For Raspberry Pi OS ros_install_generator and wstool was used rather than apt install.

## Bash Profile

Run the following commands in the terminal.

```bash
#cd ~/QVPose_catkin_ws
#source src/profile.bashrc
```

## Catkin_make

Run the following commands in the terminal.

```bash
cd ~/QVPose/catkin_ws
sudo rm -rf devel build install # or cleanWS
catkin_make -DCATKIN_BLACKLIST_PACKAGES="utari_sba_pkg" # ignore bundle adjustment package
```

# Packages and Launch Commands

List of packages and corresponding launch commands.

## vbme_pkg

This package contains the code for the QuEst algorithm in several forms and versions.

### QuEstTestFlowNoImages.launch

This is the most commonly used launch file in this package. This launch file start QuEst with no specific input. This will launch QuEst configured for the VbmeData_msg message which is meant for a serialized data flow. Input should be configured in a higher level launch file else where.

### QuEstTestFlowNoImagesOld.launch

This is a depeciated launch file. This launch file with start QuEst with no specific input. This will launch QuEst configured for the FlowMatchList_msg message which is meant for a parallel data flow. Input should be configured in a higher level launch file else where.

### QuEstTestData.launch

This launch file will run QuEst configured for the VbmeData_msg message which is meant for a serialized data flow. A simple image publisher and feature extracter will also be launched to feed input to QuEst.

### QuEstTestFlow.launch

## optical_flow - VEst

## vbme_heading_pkg

## vbme_msgs

## kalman_filter

## testing_and_analyzing

## utari_sba_pkg
