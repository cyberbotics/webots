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
apt install python3-pip python3-setuptools -y
pip3 install --upgrade pip
pip3 install pyclibrary


git clone https://github.com/emscripten-core/emsdk.git dependencies/emsdk

USER=$(env | grep SUDO_USER | cut -d '=' -f 2-)

./dependencies/emsdk/emsdk install latest
./dependencies/emsdk/emsdk activate latest
chown -R $USER dependencies/emsdk

WEBOTS_HOME=$(pwd)
echo 'source "'$WEBOTS_HOME'/dependencies/emsdk/emsdk_env.sh" >/dev/null 2>&1' >> /home/$USER/.bashrc
