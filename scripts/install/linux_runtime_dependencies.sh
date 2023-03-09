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
apt install --yes lsb-release g++ make libavcodec-extra libglu1-mesa libegl1 libxkbcommon-x11-dev libxcb-keysyms1 libxcb-image0 libxcb-icccm4 libxcb-randr0 libxcb-render-util0 libxcb-xinerama0 libxcomposite-dev libxtst6 libnss3
if [[ -z "$DISPLAY" ]]; then
       apt install --yes xvfb
fi

UBUNTU_VERSION=$(lsb_release -rs)
if [[ $UBUNTU_VERSION == "20.04" || $UBUNTU_VERSION == "22.04" ]]; then
       apt install --yes ffmpeg
else
       echo "Unsupported Linux version: dependencies may not be completely installed. Only the two latest Ubuntu LTS are supported."
fi
