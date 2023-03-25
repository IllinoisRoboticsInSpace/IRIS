# Ubuntu 20 Galactic on PC Installation Script
echo "Begin PC Installation Script"
echo "Installing Realsense SDK"

# Install Realsense SDK
# https://github.com/IntelRealSense/realsense-ros/tree/ros2
# Realsense ros wrapper is managed by the repo's github submodules

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
