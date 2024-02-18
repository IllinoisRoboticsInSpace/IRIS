# Prerequiste
```bash
sudo apt install ros-galactic-gazebo*
```
```bash
sudo apt install ros-galactic-robot-localization*
```
```bash
sudo apt install ros-galactic-rtabmap-ros*
```
# Configuration
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/YOUR_USERNAME/colcon_ws/src/sam_bot_description/models/
```
# Launch

### 1-Launch Simulation:
```bash
ros2 launch sam_bot_description display.launch.py
```

### 2-Launch Navigation2:
```bash
ros2 launch nav2_bringup navigation_launch.py
```

### 3-Launch SLAM (Option A):
```bash
ros2 launch slam_toolbox online_async_launch.py
```

### 3-Launch RTAB-Map (Option B):
```bash
ros2 launch rtabmap_ros turtlebot3_rgbd.launch.py
```


# Nav2 URDF Setup Tutorial - Differential Drive Robot
Tutorial code referenced in https://navigation.ros.org/setup_guides/urdf/setup_urdf.html

This package implements a URDF description for a simple differential drive robot. It includes the complete urdf files, launch files, build files and rviz configuration to replicate the tutorial in the link above