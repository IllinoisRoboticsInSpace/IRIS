printf "IMPORTANT!!!\nDO NOT LET THE UBUNTU 20 UPGRADE REBOOT!\nMAKE SURE TO TYPE 'N' TO CANCEL IT AT THE END\n"
printf "Select all default options for upgrade\n"

sudo do-release-upgrade

# Uncomment lines with tokens
sed -i '/WaylandEnable/s/^#//g' /etc/gdm3/custom.conf
sed -i '/"nvidia"/s/^#//g' /etc/X11/xorg.conf

sudo sed -i -e '/Prompt=/ s/=.*/=never/' /etc/update-manager/release-upgrades

echo "Reboot Jetson Immediately Before Continuing to part 3"
