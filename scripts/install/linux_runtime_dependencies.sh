#!/bin/bash

apt update
apt install --yes lsb-release g++ make libavcodec-extra libglu1-mesa libxkbcommon-x11-dev execstack libusb-dev libxcb-keysyms1 libxcb-image0 libxcb-icccm4 libxcb-randr0 libxcb-render-util0 libxcb-xinerama0 libxcomposite-dev libxtst6 libnss3

UBUNTU_VERSION=$(lsb_release -rs)
if [[ $UBUNTU_VERSIOM == "16.04" ]]; then
       apt install --yes libav-tools
elif [[ $UBUNTU_VERSION == "18.04" ]]; then
       apt install --yes ffmpeg
elif [[ $UBUNTU_VERSION == "20.04" ]]; then
       apt install --yes ffmpeg
else
       echo "Unsupported Linux version."
fi
