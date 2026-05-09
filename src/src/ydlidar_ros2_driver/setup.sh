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
