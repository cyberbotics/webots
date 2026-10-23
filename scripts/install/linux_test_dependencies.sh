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

install_ubuntu_test_packages() {
    alias apt='apt --option="APT::Acquire::Retries=3"'
    apt install liburdfdom-tools -y
}

install_fedora_test_packages() {
    dnf install -y urdfdom-devel
}

# Determine the operating system and call the appropriate function
case "$OS" in
    ubuntu)
        install_ubuntu_test_packages
        ;;
    fedora)
        install_fedora_test_packages
        ;;
    *)
        echo "Unsupported operating system: $OS"
        exit 1
        ;;
esac

if [[ $@ != *"--norecurse"* ]]; then
    echo "Installing runtime dependencies"
    script_full_path=$(dirname "$0")
    $script_full_path/linux_runtime_dependencies.sh
fi
