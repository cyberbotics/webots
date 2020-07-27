#!/bin/bash

sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.5-dev python3.6-dev python3.7-dev python3.8-dev

UBUNTU_VERSION=$(lsb_release -rs)
if [[ $(UBUNTU_VERSION) == "16.04" ]]; then
       sudo apt install openjdk-8-jdk
       curl -sL https://deb.nodesource.com/setup_12.x -o nodesource_setup.sh
       sudo bash nodesource_setup.sh
       sudo apt-get install nodejs
       rm nodesource_setup.sh
elif [[ $(UBUNTU_VERSION) == "18.04" ]]; then
       sudo apt install openjdk-11-jdk npm
elif [[ $(UBUNTU_VERSION) == "20.04" ]]; then
       sudo apt install openjdk-14-jdk npm
else
       echo "Unsupported Linux version."
fi

script_full_path=$(dirname "$0")
$script_full_path/linux_compilation_dependencies.sh
