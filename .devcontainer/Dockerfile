FROM ubuntu:20.04

LABEL maintainer="Daniel Philipov <dp33@illinois.edu>"

SHELL ["/bin/bash", "-c"]

####################################################################
#                          System Setup                            #
####################################################################
RUN apt-get update -y
RUN apt-get dist-upgrade -y

# System Dependencies
RUN apt-get install -y software-properties-common 
RUN apt-get update -y
RUN apt-get install -y tree 
RUN apt-get install -y curl

# ROS2 Dependencies
RUN apt-get install -y locales
RUN apt-get install -y gnupg2

# Camera Dependencies
RUN apt-get install -y lsb-release
RUN apt upgrade -y && apt update -y

# Install Realsense SDK
# https://github.com/IntelRealSense/realsense-ros/tree/ros2
# Realsense ros wrapper is managed by the repo's github submodules
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

####################################################################
#                            ROS Setup                             #
####################################################################

# Install ROS2
ARG ROS_DISTRO=galactic
ENV ROS_DISTRO=${ROS_DISTRO}

# Initialization
RUN apt update -y && apt install -y locales
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN apt update -y && apt install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update -y
RUN apt-get upgrade -y

# Install ROS Distro
RUN apt-get install -y ros-${ROS_DISTRO}-desktop
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN source /opt/ros/${ROS_DISTRO}/setup.bash

# Utilities
RUN apt-get install -y python3-colcon-common-extensions python3-rosdep
RUN rosdep init && rosdep update --rosdistro=${ROS_DISTRO}

# Nav2
RUN apt-get install -y ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup

# RTabMap
RUN apt-get install -y ros-${ROS_DISTRO}-rtabmap-ros

####################################################################
#                          Build Function                          #
####################################################################

RUN echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
RUN echo "function build() {" >> ~/.bashrc
RUN echo "if [ \$# == 0 ]; then" >> ~/.bashrc
RUN echo "  echo \"colcon build\"" >> ~/.bashrc
RUN echo "  colcon build" >> ~/.bashrc
RUN echo "else" >> ~/.bashrc
RUN echo "  echo \"colcon build --packages-select\" \$@" >> ~/.bashrc
RUN echo "  colcon build --packages-select \$@" >> ~/.bashrc
RUN echo "fi" >> ~/.bashrc
RUN echo "echo \"Sourcing install/setup.bash\"" >> ~/.bashrc
RUN echo "source ~/install/setup.bash" >> ~/.bashrc

####################################################################
#                        Protobuf Reworking                        #
####################################################################

RUN apt-get install -y wget

WORKDIR /root

ARG PROTOBUF_VERSION=21.5
ARG PROTOBUF_URL="https://github.com/protocolbuffers/protobuf/archive/refs/tags/v${PROTOBUF_VERSION}.zip"
ARG ZIP_NAME=protoc-${PROTOBUF_VERSION}
RUN wget ${PROTOBUF_URL} -O ${ZIP_NAME}.zip
RUN unzip ${ZIP_NAME}.zip
RUN rm ${ZIP_NAME}.zip
ARG PROTO_LIB_NAME="protobuf-${PROTOBUF_VERSION}"

WORKDIR /root/${PROTO_LIB_NAME}

RUN apt-get remove -y protobuf-compiler
RUN ./autogen.sh
RUN ./configure
RUN make -j$(nproc)

RUN make install
RUN ldconfig

WORKDIR /root

####################################################################
#                          Workspace Setup                         #
####################################################################

RUN apt-get install -y git

RUN mkdir -p /root/colcon_ws/src

WORKDIR /root/colcon_ws/src

RUN git clone https://github.com/IllinoisRoboticsInSpace/IRIS.git

####################################################################
#                             ROS Setup                            #
####################################################################

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN rosdep install -y --from-paths . --ignore-src --skip-keys=librealsense2 -r --rosdistro ${ROS_DISTRO} --include-eol-distros --os=${OS} -y
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install

####################################################################
#                             GUI Setup                            #
####################################################################

RUN apt-get install -y mesa-utils libgl1-mesa-glx
