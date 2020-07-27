#!/bin/bash

sudo apt update
sudo apt install g++ make libavcodec-extra libglu1-mesa libxkbcommon-x11-dev execstack libusb-dev libxcb-keysyms1 libxcb-image0 libxcb-icccm4 libxcb-randr0 libxcb-render-util0 libxcb-xinerama0

UBUNTU_VERSION=$(lsb_release -rs)
if [[ $(UBUNTU_VERSION) == "16.04" ]]; then
       sudo apt install libav-tools
elif [[ $(UBUNTU_VERSION) == "18.04" ]]; then
       sudo apt install ffmpeg
else
elif [[ $(UBUNTU_VERSION) == "20.04" ]]; then
       sudo apt install ffmpeg
else
       echo "Unsupported Linux version."
fi
