# Ubuntu 20 Galactic on PC Installation Script
# Will edit the ~/.bashrc, but it will not check if it was previously written to.
#
#   ONLY RUN THIS SCRIPT ONCE
#

# Update distribution and components such as kernel
# If Realsense SDK install kernel modules fail to install
# then restart device before installing realsense sdk
sudo apt-get update
sudo apt-get dist-upgrade

echo "Begin PC Installation Script"
echo "Installing Realsense SDK"

# Install Realsense SDK
# https://github.com/IntelRealSense/realsense-ros/tree/ros2
# Realsense ros wrapper is managed by the repo's github submodules

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg


# Install Arduino 2 IDE
cwd=$(pwd)
cd ~/
wget -c https://downloads.arduino.cc/arduino-ide/arduino-ide_2.0.4_Linux_64bit.AppImage -O ArduinoIDE2.AppImage
sudo chmod +x ArduinoIDE2.AppImage

# Install Sabertooth
sudo apt-get install unzip
mkdir -p ~/Arduino/libraries
cd ~/Arduino/libraries
wget -c https://www.dimensionengineering.com/software/SabertoothArduinoLibraries.zip
unzip SabertoothArduinoLibraries.zip
rm SabertoothArduinoLibraries.zip
cd $cwd

# Adds 16GB of swap for initial compilation
echo "Adding Swap"
sudo swapoff /swapfile
sudo rm  /swapfile
sudo dd if=/dev/zero of=/swapfile bs=1M count=8192 #8GB
# sudo dd if=/dev/zero of=/swapfile bs=1M count=16384 #16GB
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Set Number of Processors For Make
# Extracts amount of available memory in GB
avail_mem=$(free -t -g | grep -oP '\d+' | sed '6!d')
echo "$avail_mem GB of available memory"

echo "Setting MAKEFLAGS"
if [ $avail_mem -ge 8 ]; then
  echo "Using All CPUs"
  export MAKEFLAGS="-j$(nproc)"
else
  echo "Using 1 CPU"
  export MAKEFLAGS="-j1"
fi

./install_env_deps.sh