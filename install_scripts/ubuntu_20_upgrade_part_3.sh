sudo rm -rf /usr/share/vulkan/icd.d

sudo apt-get -y update
sudo apt-get -y upgrade
sudo apt-get -y autoremove

sudo rm /usr/share/applications/vpi1_demos

cwd=$(pwd)
cd /usr/share/nvpmodel_indicator
sudo mv nv_logo.svg no_logo.svg
cd $cwd

source_folder="/etc/apt/sources.list.d"
sudo sed -i '/cuda-repo/s/^#//g' $source_folder/cuda-*.list
sudo sed -i '/jetson/s/^#//g' $source_folder/nvidia-l4t*.list
sudo sed -i '/visionworks/s/^#//g' $source_folder/visionworks-*.list

# Add gcc and g++ alternatives
sudo apt-get install gcc-8 g++-8
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8

# if you want to make a selection use these commands
# sudo update-alternatives --config gcc
# sudo update-alternatives --config g++

# It is recommended to set gcc manually to gcc 8 as it is used for compiling CUDA on the Jetson.

# Setup ZRAM
# https://qengineering.eu/install-opencv-4.5-on-jetson-nano.html
sudo apt-get -y install dphys-swapfile
sudo sed -i -e '/CONF_MAXSWAP=/ s/=.*/=4096/' /sbin/dphys-swapfile

# Added jtop
sudo apt install python3-pip
sudo -H pip install -U jetson-stats

echo "Reboot Jetson To Finished Upgrade"

# Extra Notes
# Disable the Power Saving Blank Screen option in the settings as it gives unstable results when rebooting from sleep mode.
# It is found in settings under "Power".
