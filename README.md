# IRIS

## Getting Started

### Workspace setup

1. Create a `colcon_ws` folder on your computer
2. `cd` into it
3. Clone IRIS repo:
```bash
git clone https://github.com/IllinoisRoboticsInSpace/IRIS IRIS
```

### Docker Setup

1. Install Docker
    - You may do this any way you wish as long as you have access to the Docker Engine and CLI.
    - If in doubt, download [Docker Desktop](https://www.docker.com/)
2. Checkout IRIS repo `docker` branch
3. Create Docker container and image + make first connection
    - Run `docker build -t iris-image /path/to/your/IRIS/repo` to create an image from the Dockerfile
    - Run `docker run -it --name iris -v /path/to/your/colcon_ws:/workspaces/colcon_ws iris-image` to create a container from the `iris-image` image
        - You should now be in a remote shell in the new container
    - Run `exit` in the remote shell to close the connection
4. Connect to container
    - Run `docker start -ai iris` to open container
        - This is what you will do whenever you want to reconnect to the container in the future, don't create a new container and image each time
    - Changes you make to `/workspaces/colcon_ws` in the container will be reflected on your computer's `colcon_ws` folder and vice versa
5. Develop as you wish (e.g. open your `colcon_ws` folder in VS Code)
6. You can open a new terminal in the container with `docker exec -it iris bash`
> Note: It is generally advised to run most `git` commands from a shell connected to your computer rather than the container.

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
