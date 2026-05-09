#!/bin/sh

echo "will add user to dialout, plugdev"
echo "will install ydlidar sdk"
sudo sleep 1

mkdir -p .deps
cd .deps

sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER

if [ -d "YDLidar-SDK" ]; then 
   echo "YDLidar-SDK already installed"
   exit 0
fi

git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir -p build
cd build
cmake ..
make
sudo make install

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar.rules
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-V2.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-2303.rules

service udev reload
sleep 2
service udev restart

