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

install_ubuntu_web_viewer_packages() {
    alias apt='apt --option="APT::Acquire::Retries=3"'
    apt install python3-pip python3-setuptools -y
}

install_fedora_web_viewer_packages() {
    dnf install -y python3-pip python3-setuptools
}

# Determine the operating system and call the appropriate function
case "$OS" in
    ubuntu)
        install_ubuntu_web_viewer_packages
        ;;
    fedora)
        install_fedora_web_viewer_packages
        ;;
    *)
        echo "Unsupported operating system: $OS"
        exit 1
        ;;
esac

pip3 install --upgrade pip
pip3 install pyclibrary

git clone https://github.com/emscripten-core/emsdk.git dependencies/emsdk

USER=$(env | grep SUDO_USER | cut -d '=' -f 2-)

./dependencies/emsdk/emsdk install latest
./dependencies/emsdk/emsdk activate latest
chown -R $USER dependencies/emsdk

WEBOTS_HOME=$(pwd)
echo 'source "'$WEBOTS_HOME'/dependencies/emsdk/emsdk_env.sh" >/dev/null 2>&1' >> /home/$USER/.bashrc

if [[ "$OS" == "fedora" ]]; then
    echo "WARNING: Fedora is not an officially supported OS! Dependencies may not be completely installed. Only the two latest Ubuntu LTS are supported."
fi
