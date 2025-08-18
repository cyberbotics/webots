#!/bin/bash

# exit when any command fails on CI
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

# Function to install optional dependencies on Ubuntu
install_ubuntu_optional_compilation_packages() {
    # Install add-apt-repository command
    alias apt='apt --option="APT::Acquire::Retries=3"'
    apt install --yes software-properties-common
    add-apt-repository -y ppa:deadsnakes/ppa
    apt update
    apt install --yes curl python3.7-dev python3.8-dev python3.9-dev python3.10-dev dirmngr execstack libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev libgl2ps-dev libssh-dev

    if [[ $VERSION_ID == "22.04" ]]; then
        apt install --yes openjdk-18-jdk
    elif [[ $VERSION_ID == "24.04" ]]; then
        apt install --yes openjdk-21-jdk
    else
        echo "Unsupported Linux version: dependencies may not be completely installed. Only the two latest Ubuntu LTS versions are supported."
    fi
}

# Function to install runtime dependencies on Fedora
install_fedora_optional_compilation_packages() {
    dnf install -y curl python3-devel execstack xerces-c-devel fox-devel gdal-devel proj-devel gl2ps-devel libssh-devel
    dnf install -y java-21-openjdk-devel

    if [[ $VERSION_ID -le 42 ]]; then
        dnf install -y gnupg2
    else
        dnf install -y gnupg2-dirmngr
    fi
}

# Determine the operating system and call the appropriate function
case "$OS" in
    ubuntu)
        install_ubuntu_optional_compilation_packages
        ;;
    fedora)
        install_fedora_optional_compilation_packages
        ;;
    *)
        echo "Unsupported operating system: $OS"
        exit 1
        ;;
esac

script_full_path=$(dirname "$0")
$script_full_path/linux_test_dependencies.sh --norecurse
$script_full_path/linux_compilation_dependencies.sh

if [[ -z "$SNAP" ]]; then
  $script_full_path/linux_web_viewer_dependencies.sh
fi
