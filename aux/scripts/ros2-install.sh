#! /bin/bash

# ros2 install script for version jazzy on ubuntu 24.04
# mostly just copies from https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
# Also installs additional packages that are commonly used.
# If you have a previous version of ros2 installed, you
# will have to run the following:
### $ sudo apt remove ~nros-jazzy-* && sudo apt autoremove
### $ sudo apt remove ros2-apt-source
### $ sudo apt update
### $ sudo apt autoremove
### $ sudo apt upgrade
# and also be sure to remove the old 'source /opt/ros/<ros-version>/setup.bash'
# from your .bashrc if it is there.

# check for root
if [[ "$(whoami)" != "root" ]]; then
   echo "Script must be run as root or using sudo"
   exit 1
fi

# Check for correct Ubuntu version
VERSION="$(lsb_release -c | cut -f 2)"

if [[ "$VERSION" != "noble" ]]; then
   echo ""
   echo "Wrong Operating System - need Ubuntu 24.04"
   exit 1
fi

echo "Updating package library"

apt-get update -y

echo "Installing helper programs"
apt-get install curl git vim make cmake g++ clang++ ssh htop -y

echo "Setting correct locale"
apt install locales -y
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "Adding ROS2 apt repository"
apt install software-properties-common -y
add-apt-repository universe -y

apt-get update -y

echo "Installing ros2 apt source package"
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
dpkg -i /tmp/ros2-apt-source.deb

echo "Installing ROS2 Jazzy"
apt update -y && apt install ros-dev-tools -y
apt update -y
apt upgrade -y
apt install ros-jazzy-desktop -y
apt install ros-jazzy-ros-base -y
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> /home/$SUDO_USER/.bashrc

apt update -y
apt upgrade -y

echo "Installing additional ROS2 packages"
apt-get install ros-jazzy-ros-gz -y
apt-get install ros-jazzy-slam-toolbox -y
apt-get install ros-jazzy-robot-localization -y
apt-get install ros-jazzy-rqt* -y
apt-get install ros-jazzy-rviz2 ros-jazzy-tf2-ros ros-jazzy-tf2-tools -y
apt-get install ros-jazzy-joint-state-publisher-gui -y
apt-get install ros-jazzy-xacro -y
apt-get install ros-jazzy-gazebo-msgs -y


echo "ROS2 Jazzy Succesfully installed"
echo "Open new terminal to see ros2 command"
