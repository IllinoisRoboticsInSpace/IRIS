# IRIS
## Getting Started
> **Note**
> It is highly recommended to develop in a virtual machine because our install procedures are not isolated and can be invasive. For more information on why our team uses a VM, refer to the following documentation: ([VM Documentation](VM_Documentation.md))
1. Install [VMWare Workstation Player](https://blogs.vmware.com/workstation/2024/05/vmware-workstation-pro-now-available-free-for-personal-use.html)
   - For Windows: Download VMWare Workstation Pro for personal use.
   - For Macs: Follow these [setup instructions](MAC_Setup.md)
   - You will have to make a Broadcom account in the process, but this will just require your email for verification.
2. Add [Base Ubuntu Image](https://releases.ubuntu.com/focal/) to VMware.
   - An Ubuntu image is a pre-packaged version of the Ubuntu operating system that you can download and use to install Ubuntu on a virtual machine. The “base” image is a minimal version of Ubuntu without additional customizations or software installations.
3. Clone IRIS Repository from GitHub.
4. Install dependencies with ROSdep:
    - `cd ~/colcon_ws`
      - Change the directory to the ROS workspace (`colcon_ws`), which contains our ROS packages. The `colcon_ws` folder is typically the root workspace where we develop and build ROS projects.
    - `rosdep install --from-paths src --ignore-src --skip-keys=librealsense2 -r --rosdistro $ROS_DISTRO -y --include-eol-distros`
      - `rosdep` is a command-line tool that helps you install system dependencies needed by your ROS packages (such as libraries and third-party dependencies).
      - `--from-paths src` tells rosdep to look for ROS package dependencies in the `src` directory of the current workspace.
      - `--ignore-src` tells rosdep to ignore the source code dependencies (i.e. the actual ROS package code) and only focus on the external system dependencies (such as libraries). This is useful because rosdep doesn’t need to install the code you already have in your workspace, only the missing libraries/tools that your packages depend on.
      - `--skip-keys=librealsense2` instructs rosdep to skip installing the `librealsense2` dependency as we have a custom installation of this library. `librealsense2` is used for the Intel RealSense camera.
      - `-r` tells rosdep to resolve dependencies recursively: if any of the dependencies themselves require additional packages or dependencies, rosdep will find and install those as well.
      - `--rosdistro $ROS_DISTRO` specifies the ROS distribution version we are using so that rosdep installs the dependencies compatible with that specific distribution. The `$ROS_DISTRO` variable is set in our environment to the `galactic` ROS version.
      - `-y` tells rosdep to automatically confirm and proceed with the installation of dependencies without asking for confirmation prompts.
      - `--include-eol-distros` - EOL (End of Life) ROS distributions are no longer officially supported, but this includes them in the dependency installation process. This is useful for working with older, unsupported versions of ROS or packages that are only compatible with these EOL distributions, such as galactic.
5. Build the desired package
    - `colcon build --packages-select <Package folder name>`
    - `build <Package folder name>`
        - Use `build` function defined in `~./bashrc`
6. Source the setup script so ros2 can find the packages in this workspace 
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
