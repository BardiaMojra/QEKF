#!/bin/bash



cd ~/QVPose/catkin_ws
ls -la
printf '\n\n\n\n ---->>> clean then catkin make workspace...\n\n'
sudo rm -rf devel build install # or cleanWS
ls -la1
source  ~/QVPose/setup.bashrc
printf '\n\n\n\n ---->>> run \n'
printf "                  catkin_make -DCATKIN_BLACKLIST_PACKAGES='utari_sba_pkg'\n\n"  # ignore bundle adjustment package
ls -la1


# EOF
