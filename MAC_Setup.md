# MAC M-Series VM Setup Instructions
1. Install [VMWare Fusion Pro for Personal Use v13.6.1](https://blogs.vmware.com/workstation/2024/05/vmware-workstation-pro-now-available-free-for-personal-use.html)
   - Create a Broadcom account.
   - Download and Install v13.6.1 of VMWare Fusion Pro.
2. Download the [Ubuntu 20.04 Image](https://old-releases.ubuntu.com/releases/focal/ubuntu-20.04.4-live-server-arm64.iso)
3. Open the Ubuntu Image in VMWare Fusion Pro.
   - Double Click Install from Disk or Image.
   - Use Another Image and Find the Ubuntu 20.04 Image you downloaded.
4. Setup the Image
   - Press `Enter` on "Install Ubuntu Server".
   - Press `Enter` 6 times in a row until you get to the Guided Storage Configuration, and leave Guided Storage in its default configuration.
   - Press `Down Arrow` until "Done" is highlighted and press `Enter`.
   - Keep pressing "Done" and "Continue" until you are prompted to enter your name.
   - Enter your name and a password.
   - Press "Done" until you see a list of popular server snaps.
   - Go to "docker" and press `space` to select it.
   - Press "Done" until you see "Install Complete" at the top.
   - Wait until you see "Reboot Now" at the bottom.
   - Select "Reboot", press the `esc` and wait until the terminal stops showing new lines.
   - Login and take a snaphot (You'll see "Take Snapshot" under "Snapshots" under "Virtual Machine" on the top bar).
   - Select "Restart" under "Virtual Machine" on the top bar.
