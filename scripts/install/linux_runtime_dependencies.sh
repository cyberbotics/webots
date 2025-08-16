#!/bin/bash

# Exit when any command fails on CI
if [[ ! -z "$CI" ]]; then
    set -e
fi

if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root"
    exit 1
fi

# Detect the operating system
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS=$ID
    VERSION_ID=$VERSION_ID
else
    echo "Cannot determine the operating system."
    exit 1
fi

# Function to install runtime dependencies on Ubuntu
install_ubuntu_runtime_packages() {
    alias apt='apt --option="APT::Acquire::Retries=3"'
    apt update
    apt install --yes g++ make libavcodec-extra libglu1-mesa libegl1 \
        libxkbcommon-x11-dev libxcb-keysyms1 libxcb-image0 libxcb-icccm4 libxcb-randr0 \
        libxcb-render-util0 libxcb-xinerama0 libxcomposite-dev libxtst6 libnss3 libxcb-cursor0

    if [[ -z "$DISPLAY" ]]; then
        apt install --yes xvfb
    fi

    if [[ $VERSION_ID == "22.04" || $VERSION_ID == "24.04" ]]; then
        apt install --yes ffmpeg
    else
        echo "Unsupported Linux version: dependencies may not be completely installed. Only the two latest Ubuntu LTS are supported."
    fi
}

# Function to install runtime dependencies on Fedora
install_fedora_runtime_packages() {
    dnf install -y gcc-c++ make mesa-libGLU libEGL \
        xkeyboard-config libxcb libXcomposite libXtst nss xcb-util xcb-util-image \
        xcb-util-keysyms xcb-util-renderutil xcb-util-wm xcb-util-cursor \
        ffmpeg

    if [[ -z "$DISPLAY" ]]; then
        dnf install -y xorg-x11-server-Xvfb
    fi

    echo "WARNING: Fedora is not an officially supported OS! Dependencies may not be completely installed. Only the two latest Ubuntu LTS are supported."
}

# Determine the operating system and call the appropriate function
case "$OS" in
    ubuntu)
        install_ubuntu_runtime_packages
        ;;
    fedora)
        install_fedora_runtime_packages
        ;;
    *)
        echo "Unsupported operating system: $OS"
        exit 1
        ;;
esac
