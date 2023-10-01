# IRIS
### Prerequistes
- ROS2 Galactic
## Getting Started
1. Create a colcon workspace
    - if you're colcon workspace folder does not exist: `mkdir -p ~/colcon_ws/src`
    - Make sure the `~/colcon_ws/src` subfolder exists
2. Clone this Repo
    -  Clone this repo into the `~/colcon_ws/src` subfolder
2. Install the required dependencies with rosdep
    - `cd ~/colcon_ws`
    - `rosdep install -i --from-path src --rosdistro galactic -y`
3. Build the desired package
    - `colcon build --packages-select <Package folder name>`
4. Source the setup script so ros2 can find the packages in this workspace 
    - `source ~/colcon_ws/install/setup.bash`

## Package Usage
### Navigation
```bash
ros2 launch navigation-c display.launch.py
```

### RTAB-Map SLAM
Realsense Data Command:
```bash
ros2 launch realsense2_camera rs_launch.py enable_accel:=true enable_gyro:=true unite_imu_method:=2
```
IMU Filter Command:
```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false -r /imu/data_raw:=/camera/imu
```
RTAB-Map
```bash
ros2 launch rtabmap_ros rtabmap.launch.py \rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" \frame_id:=camera_link \rgb_topic:=/camera/color/image_raw \depth_topic:=/camera/depth/image_rect_raw \camera_info_topic:=/camera/color/camera_info \approx_sync:=true \wait_imu_to_init:=true \imu_topic:=/imu/data \rviz:=false \rtabmapviz:=true
```

### End Package Folder Structure
```
~/colcon_ws/
    build/
    install/
    log/
    src/
        IRIS-2023/
```
