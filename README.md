# IRIS

## Getting Started

### Workspace setup

1. Create a `colcon_ws` folder on your computer
2. `cd` into it
3. Clone IRIS repo:
```bash
git clone https://github.com/IllinoisRoboticsInSpace/IRIS IRIS
```
4. `cd` into the repo

### Docker Setup

1. Install Docker
    - You may do this any way you wish as long as you have access to the Docker Engine and CLI.
    - If in doubt, download [Docker Desktop](https://www.docker.com/)
2. Checkout IRIS repo `docker` branch
3. Open 'colcon_ws' in Docker container
    - Using VS Code Dev Containers extension
        - Install the Dev Containers extension 
        - Open Command Palette and run `Dev Containers: Open Folder in Container`
        - Select your `colcon_ws` folder
        - Add configuration to workspace
        - Select from Dockerfile
        - Move through the rest of the menus without selecting anything
        - You can close the connection with `Dev Containers: Reopen Folder Locally`
        - From now on, just select `Dev Containers: Reopen in Container` to open folder in container
    - Using Docker CLI
        - Run `docker build -t iris-image .` to create an image from the Dockerfile
        - Run `docker run -it --name iris -v /path/to/your/colcon_ws:/workspaces/colcon_ws iris-image`
        - Run `exit` in the remote shell to reopen locally
        - From now on, run `docker start -ai iris` to open folder in container
    - Changes you make to `colcon_ws` in the container will be reflected on your computer and vice versa
4. Develop while connected to container
> Note: It is generally advise to run most `git` commands and other operations affecting the filetree from a shell connected to your computer.
> If using VS Code Dev Containers, reopen locally before adding, committing, and pushing changes.

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
