# PART 1 of ubuntu 20 upgrade script
# To easily read comments enable text wrapping.
# Alt + z in vscode to toggle text wrapping.
# This script is meant to upgrade a jetson nano dev kit from the default Ubuntu 18 base image provided by NVIDIA to Ubuntu 20 so that ROS 2 Galactic can be used.
# The upgrade process used is detailed at this link: https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html
# Another important fact is that the Jetson Nano will not be recieving more updates so Ubuntu 20 will not be supported by NVIDIA: https://forums.developer.nvidia.com/t/jetson-software-roadmap-for-2h-2021-and-2022/177721
# While premade images exist for Ubuntu 20, these images contain specific version of the Jetpack SDK so it is necessary to upgrade to Ubuntu 20 from scratch: https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image
# First flash an SD card with an image from NVIDIA with the desired version of Jetpack SDK.
# Select an file version of name "Jetson Nano Developer Kit SD Card Image" from the following links.
# Latest Versions: https://developer.nvidia.com/embedded/downloads
# Older Version: https://developer.nvidia.com/embedded/downloads/archive

# First feel free to uninstall any applications from the software updater before continuing

sudo apt-get remove --purge chromium-browser chromium-browser-l10n
sudo apt-get install firefox
sudo apt-get -y update
sudo apt-get -y upgrade
sudo apt-get -y autoremove

sudo sed -i -e '/Prompt=/ s/=.*/=normal/' /etc/update-manager/release-upgrades

cat /etc/update-manager/release-upgrades

sudo apt-get update
sudo apt-get dist-upgrade
echo "Reboot Jetson Immediately Before Continuing to part 2"

# Extra notes:
# The following problem was encountered on the jetpack 4.5.1 image. If there is a dpkg subprocess error with nvidia-l4t-bootloader run the commands found in this link: https://forums.developer.nvidia.com/t/solution-dpkg-error-processing-package-nvidia-l4t-bootloader-configure/208627.