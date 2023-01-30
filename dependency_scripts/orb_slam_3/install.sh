#!/bin/bash
# Author: Teodor Tchalakov 1/28/2023
# To run do:
# 'chmod +x install.sh'
# './install.sh'
#
# Place script in an empty folder and run with sudo.
mkdir -p ~/orb_slam3
cd ~/orb_slam3
sudo apt-get install -y wget nasm python3-pip
pip install numpy
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin/
./scripts/install_prerequisites.sh -m apt all
cmake -DPython_EXECUTABLE=`which python3` -B build
cmake --build build -j$(nproc)
cmake --build build -t pypangolin_pip_install
sudo cmake --install build
cd ../
# Attempting to install using vcpkg, did not work
#git clone https://github.com/Microsoft/vcpkg.git
#./vcpkg/bootstrap-vcpkg.sh
#./vcpkg/vcpkg install pangolin
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.7.0.zip
unzip opencv.zip
cd opencv-4.7.0
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
cd ../..
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar -xvf eigen-3.4.0.tar.gz
cd eigen-3.4.0/
mkdir -p build && cd build
cmake ..
sudo make install
cd ../..
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3/
sed -i 's/++11/++14/g' CMakeLists.txt
sed -i 's/j4/j1/g' build.sh
chmod +x build.sh
sudo ./build.sh
cd ../
echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$PWD/ORB_SLAM3/Examples/ROS" >> ~/.bashrc
cd ORB_SLAM3/
chmod +x build_ros.sh
sudo ./build_ros.sh
cd ../
echo "FINISHED"