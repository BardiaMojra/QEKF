#!/bin/bash

# config
_cv_version="3.4.9"
_project_name='QVPose' # used as sub-root directory folder

printf '\n\n\n\n ---->>> setting up opencv-%s source files...\n\n' ${_cv_version}
# setup

## dependencies
sudo apt update
sudo apt-get install build-essential -y # compiler
sudo apt-get install cmake -y
sudo apt-get install git -y
sudo apt-get install libgtk2.0-dev  -y
sudo apt-get install pkg-config  -y
sudo apt-get install libavcodec-dev -y
sudo apt-get install libavformat-dev  -y
sudo apt-get install libswscale-dev -y  # required dependencies
sudo apt-get install python-dev -y
sudo apt-get install python-numpy -y
sudo apt-get install libtbb2 -y
sudo apt-get install libtbb-dev -y
sudo apt-get install libjpeg-dev -y
sudo apt-get install libpng-dev -y
sudo apt-get install libtiff-dev -y
sudo apt-get install libjasper-dev  -y
sudo apt-get install libdc1394-22-dev -y # optional dependencies


cd ~/${_project_name}/opencv_setup

## if $1 is not passed, set to the current working dir using $PWD
#_dir="${1:-${PWD}/opencv-${_cv_version}}"

## if exist
#[  -d "$_dir" ] && { printf "\n\n \--->>> Warning: Directory $_dir will be deleted."; sudo rm -rfv opencv-${_cv_version}; printf '\n\n' ;}

sudo rm -rfv opencv-${_cv_version}
mkdir opencv-${_cv_version} && sudo chown ${USER:=$(/usr/bin/id -run)}:$USER opencv-${_cv_version}
cd opencv-$_cv_version

## get source files
#git clone https://github.com/opencv/opencv.git
#git -C opencv checkout ${_cv_version}
#sudo rm -rfv opencv-${_cv_version}/opencv/.git*

#git clone https://github.com/opencv/opencv_contrib.git
#git -C opencv_contrib checkout ${_cv_version}
#sudo rm -rfv opencv-${_cv_version}/opencv_contrib/.git*

## setup submodules

git submodule add --force https://github.com/opencv/opencv.git opencv
cd opencv && git fetch
git checkout ${_cv_version}
cd ..
git submodule add --force https://github.com/opencv/opencv_contrib.git opencv_contrib
cd opencv_contrib && git fetch
git checkout ${_cv_version}
cd .. && tree -L 2

printf '\n\n\n\n ---->>> opencv setup finished, now run build_opencv.sh...\n\n'

# EOF
