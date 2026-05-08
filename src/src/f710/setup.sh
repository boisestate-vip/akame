#!/bin/sh

# does the setup tasks listed in the readme.

# get sudo
echo "getting sudo..."
sudo sleep 1
echo "sudo acquired!"

echo "copying udev rules..."
sudo cp 90-hidraw-permissions.rules /etc/udev/rules.d/
echo "udev rules copied!"

echo "reloading udevadm..."
sudo udevadm control --reload-rules 
sudo udevadm trigger
echo "udevadm reloaded!"

echo "adding user to plugdev group..."
sudo usermod -aG plugdev $USER
echo "user added to plugdev!"

echo "done!"
