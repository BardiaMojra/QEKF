#!/bin/bash

# config
_cv_version="3.4.9"
_project_name='QVPose' # used as sub-root directory folder
exit 0 # doesnt work
printf '\n\n\n\n ---->>> building opencv-%s from source...\n\n' ${_cv_version}
_src_path='~/${_project_name}/opencv_setup/opencv-${_cv_version}/opencv'
cd ${_src_path} && printf '\n\n \--->> PWD: $s' ${_src_path}
sudo rm -rf build && mkdir build
sudo chown ${USER:=$(/usr/bin/id -run)}:$USER build # may have to run in terminal
ls -al1 --color=always && cd build

exit 0

## config cmake
# cmake -D CMAKE_BUILD_TYPE=Release \
#            -D CMAKE_INSTALL_PREFIX=/usr/local \
#            -D BUILD_EXAMPLES=ON \
#            -D BUILD_DOCS=ON \
#            ..

cmake -D CMAKE_BUILD_TYPE=RELEASE \
           -D CMAKE_INSTALL_PREFIX=/usr/local \
           -D INSTALL_PYTHON_EXAMPLES=ON \
           -D INSTALL_C_EXAMPLES=ON \
           -D OPENCV_EXTRA_MODULES_PATH=/home/smerx/QVPose/opencv_setup/opencv-3.4.9/opencv_contrib/modules \
           -D PYTHON_EXECUTABLE=$(which python) \
           -D BUILD_EXAMPLES=ON \
           -D BUILD_DOCS=ON \
           ..

## make the project in build directory
make -j8

## make documentation via doxygen
#cd doc && sudo make -j8 doxygen

## make install in build directory
#cd ..
sudo make install

# EOF
