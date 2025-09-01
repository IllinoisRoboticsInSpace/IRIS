# Docker Documentation
The installation process for the required software, tools, and dependencies (like ROS, motor drivers, hardware interfaces, etc.) may modify or overwrite critical system files, settings, and dependencies on your main operating system. These changes can affect other applications or development environments on your main machine, possibly causing conflicts or breaking other software.
### Why Use a Docker Container
- **Isolation:** A Docker container creates an isolated environment that behaves like a seperate OS. Any changes made within the Docker container will not affect your main operating system.
- **Safety:** If something goes wrong (e.g. installation errors, package conflicts, or corrupted settings), it only affects the container, not your entire computer.
- **Consistency:** The Docker container is pre-configured to have the exact environment needed for the project (like Ubuntu, ROS, etc.), ensuring that everything works together without compatibility issues.
