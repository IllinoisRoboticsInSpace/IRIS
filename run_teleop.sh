#!/bin/bash

# Define the ROS commands to run
if [ -n "$1" ]; then
    PORT=$1
else
    PORT="/dev/ttyACM0"
fi

SOURCE_ROS="source /opt/ros/galactic/setup.bash"
SOURCE_INSTALL="source ~/colcon_ws/install/setup.bash"
LAUNCH_TELEOP="ros2 launch teleop teleop_launch.py"
# LAUNCH_DRIVE="ros2 launch tank_control robot_controller_launch.py"
# LAUNCH_LIN_ACT="ros2 launch cytron_linear_actuator cytron_linear_actuator.launch.py"

LOCAL_COMMAND="$SOURCE_ROS ; $SOURCE_INSTALL ; $LAUNCH_TELEOP"
REMOTE_COMMAND1="~/colcon_ws/src/IRIS/launch_drive.sh"
REMOTE_COMMAND2="~/colcon_ws/src/IRIS/launch_lin_act.sh"

# Remote machine details
REMOTE_HOST="192.168.1.135" 
REMOTE_USER="iris-jetson"     

# Check which terminal emulator is available
if command -v gnome-terminal &> /dev/null; then
    # For GNOME Terminal (Ubuntu, etc.)
    gnome-terminal -- bash -c "$LOCAL_COMMAND; exec bash"
    gnome-terminal -- bash -c "ssh -t $REMOTE_USER@$REMOTE_HOST '$REMOTE_COMMAND1'; exec bash"
    gnome-terminal -- bash -c "ssh -t $REMOTE_USER@$REMOTE_HOST '$REMOTE_COMMAND2'; exec bash"
else
    echo "No supported terminal emulator found."
    exit 1
fi

echo "All ROS nodes launched. Teleop running locally, tank_control and cytron_linear_actuator running on $REMOTE_HOST"

read 

killall gnome-terminal-