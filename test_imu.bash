source /opt/ros/galactic/setup.bash

cd ~/colcon_ws
colcon build --base-paths ./src/CameraTesting

source ./install/setup.bash

gnome-terminal \
    --tab -- bash -c \
    "ros2 launch realsense2_camera rs_launch.py \
    e ;bash"
    # "ros2 launch realsense2_camera rs_d455_launch.py ;bash"
gnome-terminal \
    --tab -- bash -c \
    "ros2 run rs_imu rs_imu ;bash" 
gnome-terminal \
    --tab -- bash -c \
    "ros2 topic echo /camera/imu ;bash"

read
killall gnome-terminal-
