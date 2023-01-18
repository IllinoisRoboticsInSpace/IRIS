# IRIS-2023
### Prerequistes
- ROS2 Humble
### Getting Started
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

### Package Usage
#TODO
### End Package Folder Structure
```
~/colcon_ws/
    build/
    install/
    log/
    src/
        IRIS-2023/
```
