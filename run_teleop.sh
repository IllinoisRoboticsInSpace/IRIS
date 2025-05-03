#!/bin/bash

# Define the ROS commands to run
LOCAL_COMMAND="source /opt/ros/galactic/setup.bash && ros2 launch teleop teleop_launch.py"
REMOTE_COMMAND1="source /opt/ros/galactic/setup.bash && ros2 launch tank_control robot_controller_launch.py"
REMOTE_COMMAND2="source /opt/ros/galactic/setup.bash && ros2 launch cytron_linear_actuator cytron_linear_actuator.launch.py"

# Remote machine details
REMOTE_HOST="192.168.1.135" 
REMOTE_USER="iris-jetson"     

# Check which terminal emulator is available
if command -v gnome-terminal &> /dev/null; then
    # For GNOME Terminal (Ubuntu, etc.)
    gnome-terminal -- bash -c "$LOCAL_COMMAND; exec bash"
    gnome-terminal -- bash -c "ssh $REMOTE_USER@$REMOTE_HOST '$REMOTE_COMMAND1'; exec bash"
    gnome-terminal -- bash -c "ssh $REMOTE_USER@$REMOTE_HOST '$REMOTE_COMMAND2'; exec bash"
else
    echo "No supported terminal emulator found."
    exit 1
fi

echo "All ROS nodes launched. Teleop running locally, tank_control and cytron_linear_actuator running on $REMOTE_HOST"