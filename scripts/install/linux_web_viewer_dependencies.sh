#!/bin/bash

# exit when any command fails on CI
if [[ ! -z "$CI" ]]; then
       set -e
fi

if [[ $EUID -ne 0 ]]; then
       echo "This script must be run as root"
       exit 1
fi

WEBOTS_HOME="$(cd "$(dirname "${BASH_SOURCE[0]}" )"/../.. && pwd)"
EMSDK_VERSION=6.0.9

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
    apt install python3-pip python3-setuptools python3-venv -y
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

TARGET_USER="${SUDO_USER:-root}"
TARGET_GROUP=$(id -gn "$TARGET_USER")

# pyclibrary is installed in a local virtual environment: recent Linux distributions mark
# the system Python installation as externally managed (PEP 668) and refuse a plain pip install.
PYCLIBRARY_VENV="$WEBOTS_HOME/dependencies/pyclibrary-venv"
python3 -m venv "$PYCLIBRARY_VENV"
"$PYCLIBRARY_VENV/bin/python3" -m pip install --no-input pyclibrary
chown -R "$TARGET_USER":"$TARGET_GROUP" "$PYCLIBRARY_VENV"

# Emscripten is pinned to a known working version; both emsdk commands are idempotent
EMSDK_PATH="$WEBOTS_HOME/dependencies/emsdk"
if [ ! -d "$EMSDK_PATH" ]; then
  git clone https://github.com/emscripten-core/emsdk.git "$EMSDK_PATH"
fi
"$EMSDK_PATH/emsdk" install "${EMSDK_VERSION}" && "$EMSDK_PATH/emsdk" activate "${EMSDK_VERSION}"
if [ ! -x "$EMSDK_PATH/upstream/emscripten/emcc" ]; then
  echo "Failed to install EMSDK ${EMSDK_VERSION}"
  exit 1
fi
chown -R "$TARGET_USER":"$TARGET_GROUP" "$EMSDK_PATH"

if [[ "$OS" == "fedora" ]]; then
    echo "WARNING: Fedora is not an officially supported OS! Dependencies may not be completely installed. Only the two latest Ubuntu LTS are supported."
fi
