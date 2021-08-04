#!/bin/bash

# source bash
source /opt/ros/melodic/setup.bash

# workspace aliases
alias cleanWS='rm -rf build devel install'

# --------------- cmake ---------------
export CMAKE_INCLUDE_PATH=/usr/local/include:/usr/include/hdf5/serial:/usr/local/include/opencv2/hdf:$CMAKE_INCLUDE_PATH
export CMAKE_LIBRARY_PATH=/usr/local/lib:/usr/include/hdf5/serial:/usr/local/include/opencv2/hdf:$CMAKE_LIBRARY_PATH
export CMAKE_PREFIX_PATH=/usr/include/hdf5/serial:/usr/local/include/opencv2/hdf:$CMAKE_PREFIX_PATH


# EOF
