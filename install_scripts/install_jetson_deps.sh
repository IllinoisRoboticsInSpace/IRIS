# Script to install dependencies and configure the jetson nano
# Only meant to be run once

sudo apt-get update
sudo apt-get dist-upgrade

# Setup CUDA Library Install
# cuda 10.2
# Link: https://github.com/jetsonhacks/buildLibrealsense2TX/issues/13#issuecomment-573976359
echo "export CUDA_HOME=/usr/local/cuda" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64" >> ~/.bashrc
echo "export PATH=$PATH:$CUDA_HOME/bin" >> ~/.bashrc

# Setup local network ROS communication
echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc

export CUDA_HOME=/usr/local/cuda
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64
export PATH=$PATH:$CUDA_HOME/bin

# Adds 16GB of swap for initial compilation
# echo "Adding Swap"

# Uncomment to remove old swap file named /swapfile
# sudo swapoff /swapfile
# sudo rm  /swapfile
# sudo dd if=/dev/zero of=/swapfile bs=1M count=16384
# sudo chmod 600 /swapfile
# sudo mkswap /swapfile
# sudo swapon /swapfile

# Not working on jetson nano ubuntu 20
# Install Realsense SDK using debian apt
# sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
# sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
# sudo apt-get install librealsense2-utils
# sudo apt-get install librealsense2-dev
# sudo apt-get install librealsense2-dkms
# sudo apt-get install librealsense2-dbg

# Install Realsense SDK
cwd=$(pwd)
cd ~/
mkdir IRIS_dependencies
cd IRIS_dependencies
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense

# Choose a backend for usb
# Install from Source Native Backend
# Link: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md#building-from-source-using-native-backend
./scripts/patch-realsense-ubuntu-L4T.sh
sudo apt-get install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev -y
./scripts/setup_udev_rules.sh
mkdir build
cd build
cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=false -DBUILD_WITH_CUDA=true && make -j$(($(nproc)-1)) && sudo make install

# Install RSUSB Backend
# ./scirpts/libuvc_installation.sh
cd $cwd

# Set Number of Processors For Make
# Must be small for memory intensive compilation
export MAKEFLAGS="-j1"

./install_env_deps.sh