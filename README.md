# IRIS

## Getting Started

### Workspace setup

1. Create a `colcon_ws` folder on your computer
2. Create an `src` folder inside `colcon_ws`
3. `cd` into it
    - If you run `pwd` the output should end with `colcon_ws/src`
4. Clone IRIS repo:
```bash
git clone https://github.com/IllinoisRoboticsInSpace/IRIS IRIS
```
6. `cd` into it
    - If you run `pwd` the output should end with `colcon_ws/src/IRIS`

### Docker Setup

1. Install Docker
    - You may do this any way you wish as long as you have access to the Docker Engine and CLI.
    - If in doubt, download [Docker Desktop](https://www.docker.com/)
2. Checkout IRIS repo `docker` branch
3. Pick one of the two options below

#### Manually Create Container and Image

1. Creation and intial connection
    - Run `docker build -t iris-image /path/to/your/IRIS/repo` to create an image from the Dockerfile
    - Run `docker run -it --name iris -v /path/to/your/colcon_ws:/workspaces/colcon_ws iris-image` to create a container from the `iris-image` image
        - You should now be in a remote shell in the new container
    - Run `exit` in the remote shell to close the connection
2. Connecting in the future
    - Run `docker start -ai iris` to open container
        - This is what you will do whenever you want to reconnect to the container in the future, don't create a new container and image each time
    - You can open a new terminal connected to the container with `docker exec -it iris bash` from your computer's terminal
    - Changes you make to `/workspaces/colcon_ws` in the container will be reflected on your computer's `colcon_ws` folder and vice versa
3. Develop as you wish (e.g. open your `colcon_ws` folder in VS Code or editor of choice)

#### Use VS Code Dev Containers

1. Copy the `.devcontainer` folder into `colcon_ws`
2. Open `colcon_ws` in VS Code
3. Install the `Dev Containers` extension
4. Run the VS Code command `Dev Containers: Reopen in Container`
    - You can access the command palette with `Ctrl+Shift+P`

> Note: It is generally advised to run most `git` commands from a terminal connected to your computer rather than the container

### End Package Folder Structure

```
~/colcon_ws/
    build/
    install/
    log/
    src/
        IRIS/
```

## GitHub Guidelines

### Contributing Code

- Commits should be well-named and modular
- Commit to the correct branch
- Pushing code into the `comp/*` and `*/main-YYYY` branches will be blocked by GitHub - push code to a feature branch and open a pull request when it is ready to be merged

### Branch Naming Scheme

Unless branch names are in one of the following formats, they will be blocked by GitHub. Please adhere to the naming scheme below.

- `comp/*` - final competition code for that year like `comp/2027`
- `perception/*` - final code for perception like `perception/main-2027` or feature code for perception like `perception/apriltags-2027`
- `navigation/*` - final code for navigation like `navigation/main-2027` or feature code for navigation like `navigation/a-star-2027`
- `embedded/*` - final code for embedded like `embedded/main-2027` or feature code for embedded like `embedded/linear-actuator-2027`
- `controls/*` - final code for controls like `controls/main-2027` or feature code for controls like `controls/joystick-2027`
- `simulation/*` - final code for simulation like `simulation/main-2027` or feature code for simulation like `simulation/gazebo-2027`
- `hotfix/*` - hotfixes to be applied to `comp/*` and `*/main-YYYY` branches
