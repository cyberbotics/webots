#!/bin/bash

sudo apt update
sudo apt install git cmake swig python2.7-dev libglu1-mesa-dev libglib2.0-dev libfreeimage-dev libfreetype6-dev libxml2-dev libzzip-0-13 libboost-dev libgd3 libssh-gcrypt-dev libzip-dev libreadline-dev pbzip2 libpci-dev

UBUNTU_VERSION=$(lsb_release -rs)
if [[ $(UBUNTU_VERSION) == "16.04" ]]; then
       sudo apt install libssl-dev python-pip
elif [[ $(UBUNTU_VERSION) == "18.04" ]]; then
       sudo apt install libssl1.0-dev python-pip
elif [[ $(UBUNTU_VERSION) == "20.04" ]]; then
       sudo apt install libssl-dev libzip5 python-pip-whl
else
       echo "Unsupported Linux version."
fi

script_full_path=$(dirname "$0")
$script_full_path/linux_runtime_dependencies.sh
