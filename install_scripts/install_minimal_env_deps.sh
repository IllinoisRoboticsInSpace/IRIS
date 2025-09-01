# Script to automatically install all IRIS env dependencies
# Will edit the ~/.bashrc, but it will not check if it was previously written to.
#
#   ONLY RUN THIS SCRIPT ONCE
#

echo "Setting Up IRIS Galactic Development Environment Dependencies"

# Install ROS 2 Galactic
# Initialization
sudo apt update -y && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt update -y && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update -y
sudo apt upgrade -y

# Install Galactic
sudo apt install -y ros-galactic-desktop
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
source /opt/ros/galactic/setup.bash

# Utilities
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init # Can fail if already initialized previously, therefore disregard error
rosdep update --rosdistro=$ROS_DISTRO

# Extra Installation Instructions
echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc