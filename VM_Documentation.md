# VM Documentation
The installation process for the required software, tools, and dependencies (like ROS, motor drivers, hardware interfaces, etc.) may modify or overwrite critical system files, settings, and dependencies on your main operating system. These changes can affect other applications or development environments on your main machine, possibly causing conflicts or breaking other software.
### Why Use a Virtual Machine
- **Isolation:** A VM creates an isolated environment that behaves like a separate computer. Any changes made within the VM (like installing ROS or configuring drivers) will not affect your main operating system.
- **Safety:** If something goes wrong (e.g. installation errors, package conflicts, or corrupted settings), it only affects the virtual environment, not your entire computer. You can also revert the VM to a previous snapshot if needed.
- **Consistency:** The VM is often pre-configured to have the exact environment needed for the project (like Ubuntu for ROS development), ensuring that everything works together without compatibility issues.
