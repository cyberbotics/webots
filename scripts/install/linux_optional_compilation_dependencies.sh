#!/bin/bash

if [[ $EUID -ne 0 ]]; then
       echo "This script must be run as root"
       exit 1
fi

# Install add-apt-repository command
apt install --yes software-properties-common
add-apt-repository -y ppa:deadsnakes/ppa
apt update
apt install --yes lsb-release curl python3.6-dev python3.7-dev python3.8-dev python3.9-dev dirmngr
curl -sL https://deb.nodesource.com/setup_15.x | bash -
apt install --yes nodejs

UBUNTU_VERSION=$(lsb_release -rs)
if [[ $UBUNTU_VERSION == "16.04" ]]; then
       apt install --yes openjdk-8-jdk python3.5-dev
       export ROS_DISTRO=kinetic
elif [[ $UBUNTU_VERSION == "18.04" ]]; then
       apt install --yes openjdk-11-jdk
       export ROS_DISTRO=melodic
elif [[ $UBUNTU_VERSION == "20.04" ]]; then
       apt install --yes openjdk-14-jdk
       export ROS_DISTRO=noetic
else
       echo "Unsupported Linux version."
fi

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt update -qq
apt-get install -y ros-$ROS_DISTRO-ros-base ros-$ROS_DISTRO-sensor-msgs ros-$ROS_DISTRO-tf liburdfdom-tools

script_full_path=$(dirname "$0")
$script_full_path/linux_compilation_dependencies.sh
