#!/bin/sh

echo "will install openvr"
echo "will turn off apparmour restrict unprivileged users"

sudo sleep 1

mkdir -p .deps
cd .deps

sudo sysctl -w kernel.apparmor_restrict_unprivileged_userns=0

if [ -d "openvr" ]; then 
   echo "openvr already installed"
   exit 0
fi

git clone https://github.com/ValveSoftware/openvr.git
cd ./openvr
mkdir build
cd ./build
cmake ..
make
sudo make install

cd ../..
echo 'export OPENVR_DIRECTORY="$(pwd)/openvr"' >> ~/.bashrc
. export OPENVR_DIRECTORY="$(pwd)/openvr"
