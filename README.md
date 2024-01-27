# IRIS
### Prerequistes
- ROS2 Galactic
- ROS2 [Video4Linux2](https://gitlab.com/boldhearts/ros2_v4l2_camera) camera driver
    - ```bash
        sudo apt install ros-$ROS_DISTRO-v4l2-camera
        ```
## Getting Started
> **Note**
> It is highly recommended to develop in a virtual machine because our install procedures are not isolated and can be invasive ([VM Instructions](url))
1. Install [VMWare Workstation Player](https://customerconnect.vmware.com/en/downloads/info/slug/desktop_end_user_computing/vmware_workstation_player/17_0)
2. Download, Unzip, and Run in VMWare our [preconfigured VM](https://uofi.app.box.com/folder/178594834739?s=xefuv04cugxavr3wadn55qbbtfs31ig4)
3. Open the `~/colcon_ws` folder in VSCode
4. If working on Arduino, install the PlatformIO VSCode extension
   
### Commands to know
1. Install dependencies with rosdep
    - `cd ~/colcon_ws`
    - `rosdep install --from-paths src --ignore-src --skip-keys=librealsense2 -r --rosdistro $ROS_DISTRO -y --include-eol-distros`
2. Build the desired package
    - `colcon build --packages-select <Package folder name>`
    - `build <Package folder name>`
        - Use `build` function defined in `~./bashrc`
3. Source the setup script so ros2 can find the packages in this workspace 
    - `source ~/colcon_ws/install/setup.bash`

### Package Usage
### AprilTags
Camera Node:
```bash
ros2 run v4l2_camera v4l2_camera_node
```
AprilTag Node:
```bash
ros2 launch iris_apriltag_ros tag_36h11_all.launch.py
```
Published Node Information (see via `ros2 topic list` cmd):
```bash
/v4l2/image_raw # image taken from camera driver
/v4l2/camera_info # camera info from camera driver, e.g. height, width, intrinsics
/apriltag/detections # tag family, tag id, corner & center (x,y) coordinates, homography, decision margin
/apriltag/tag_detections_image # visualization of four corners and coordinate axis
```

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
RTAB-Map Command:
```bash
ros2 launch rtabmap_ros rtabmap.launch.py \rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" \frame_id:=camera_link \rgb_topic:=/camera/color/image_raw \depth_topic:=/camera/depth/image_rect_raw \camera_info_topic:=/camera/color/camera_info \approx_sync:=true \wait_imu_to_init:=true \imu_topic:=/imu/data \rviz:=false \rtabmapviz:=true
```
RTAB-Map Debug Mode Command:
```bash
ros2 launch rtabmap_ros rtabmap.launch.py \rtabmap_args:="--delete_db_on_start" \frame_id:=camera_link \rgb_topic:=/camera/color/image_raw \depth_topic:=/camera/depth/image_rect_raw \camera_info_topic:=/camera/color/camera_info \approx_sync:=true \wait_imu_to_init:=true \imu_topic:=/imu/data \rviz:=true \rtabmapviz:=false \rtabmap_args:="-d --udebug" \launch_prefix:="xterm -e gdb -ex run --args"
```

### End Package Folder Structure
```
~/colcon_ws/
    build/
    install/
    log/
    src/
        IRIS/
```
