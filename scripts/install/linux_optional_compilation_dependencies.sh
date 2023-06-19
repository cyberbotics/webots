#!/bin/bash

# exit when any command fails on CI
if [[ ! -z "$CI" ]]; then
       set -e
fi

if [[ $EUID -ne 0 ]]; then
       echo "This script must be run as root"
       exit 1
fi

# Install add-apt-repository command
alias apt='apt --option="APT::Acquire::Retries=3"'
apt install --yes software-properties-common
add-apt-repository -y ppa:deadsnakes/ppa
apt update
apt install --yes lsb-release curl python3.7-dev python3.8-dev python3.9-dev python3.10-dev dirmngr execstack libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev libgl2ps-dev libssh-dev

UBUNTU_VERSION=$(lsb_release -rs)
if [[ $UBUNTU_VERSION == "22.04" ]]; then
       apt install --yes openjdk-18-jdk
elif [[ $UBUNTU_VERSION == "20.04" ]]; then
       apt install --yes openjdk-16-jdk
else
       echo "Unsupported Linux version: dependencies may not be completely installed. Only the two latest Ubuntu LTS are supported."
fi

script_full_path=$(dirname "$0")
$script_full_path/linux_test_dependencies.sh --norecurse
$script_full_path/linux_compilation_dependencies.sh
$script_full_path/linux_web_viewer_dependencies.sh
