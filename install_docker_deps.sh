# Script to automatically install all IRIS env dependencies
# Will edit the ~/.bashrc, but it will not check if it was previously written to.
#
#   ONLY RUN THIS SCRIPT ONCE
#

echo "Setting Up IRIS Galactic Development Environment Dependencies"

export DEBIAN_FRONTEND=noninteractive

# # Install ROS 2 Galactic
# # Initialization
# apt update -y && apt install -y locales
# locale-gen en_US en_US.UTF-8
# update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# export LANG=en_US.UTF-8
# apt update -y && apt install -y curl gnupg lsb-release
# curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
# apt update -y
# apt upgrade -y
#
# # Install Galactic
# apt install -y ros-galactic-desktop
# echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
# source /opt/ros/galactic/setup.bash
#
# # Utilities
# apt-get install -y python3-colcon-common-extensions python3-rosdep
# rosdep init # Can fail if already initialized previously, therefore disregard error
# rosdep update --rosdistro=$ROS_DISTRO

apt update
apt install -y curl gnupg2 lsb-release

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

apt update

# Nav2
apt install -y ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup

# Install RTAB-MAP
# https://github.com/introlab/rtabmap_ros/tree/ros2#binaries
apt-get install -y ros-$ROS_DISTRO-rtabmap-ros

# Extra Installation Instructions
echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc

echo "function build() {
if [ \$# == 0 ]; then
  echo "colcon build"
  colcon build
else 
  echo "colcon build --packages-select" \$@
  colcon build --packages-select \$@
fi
echo "Sourcing install/setup.bash"
source ./install/setup.bash
}" >> ~/.bashrc

# Install April Tags
# cwd=$(pwd)
# cd ~/
# mkdir -p IRIS_dependencies
# cd IRIS_dependencies
# git clone https://github.com/AprilRobotics/apriltag.git
# cd apriltag
# cmake -B build -DCMAKE_BUILD_TYPE=Release
# cmake --build build --target install
# cd $cwd
