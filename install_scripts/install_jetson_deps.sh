# Script to install dependencies and configure the jetson nano
# Only meant to be run once

# Adds 16GB of swap for initial compilation
sudo swapoff /swapfile
sudo rm  /swapfile
sudo dd if=/dev/zero of=/swapfile bs=1M count=16384
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile