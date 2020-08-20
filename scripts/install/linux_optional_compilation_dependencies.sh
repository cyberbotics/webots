#!/bin/bash

sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.5-dev python3.6-dev python3.7-dev python3.8-dev

UBUNTU_VERSION=$(lsb_release -rs)
if [[ $UBUNTU_VERSION == "16.04" ]]; then
       sudo apt install openjdk-8-jdk
       curl -sL https://deb.nodesource.com/setup_12.x -o nodesource_setup.sh
       sudo bash nodesource_setup.sh
       sudo apt-get install nodejs
       rm nodesource_setup.sh
       export ROS_DISTRO=kinetic
elif [[ $UBUNTU_VERSION == "18.04" ]]; then
       sudo apt install openjdk-11-jdk npm
       export ROS_DISTRO=melodic
elif [[ $UBUNTU_VERSION == "20.04" ]]; then
       sudo apt install openjdk-14-jdk npm
       export ROS_DISTRO=noetic
else
       echo "Unsupported Linux version."
fi

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update -qq
sudo apt-get install -y ros-$ROS_DISTRO-ros-base ros-$ROS_DISTRO-sensor-msgs ros-$ROS_DISTRO-tf liburdfdom-tools

script_full_path=$(dirname "$0")
$script_full_path/linux_compilation_dependencies.sh
