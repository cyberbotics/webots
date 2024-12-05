#!/bin/bash

# exit when any command fails on CI
if [[ ! -z "$CI" ]]; then
       set -e
fi

if [[ $EUID -ne 0 ]]; then
       echo "This script must be run as root"
       exit 1
fi

alias apt='apt --option="APT::Acquire::Retries=3"'
apt update
apt install --yes git lsb-release cmake swig libglu1-mesa-dev libglib2.0-dev libfreeimage3 libfreetype6-dev libxml2-dev libboost-dev libssh-gcrypt-dev libzip-dev libreadline-dev pbzip2 wget zip unzip python3 python3-pip libopenal-dev

UBUNTU_VERSION=$(lsb_release -rs)
if [[ $UBUNTU_VERSION == "22.04" || $UBUNTU_VERSION == "24.04" ]]; then
       apt install --yes libzip4 openssl
else
       echo "Unsupported Linux version: dependencies may not be completely installed. Only the two latest Ubuntu LTS are supported."
fi

script_full_path=$(dirname "$0")
$script_full_path/linux_runtime_dependencies.sh
