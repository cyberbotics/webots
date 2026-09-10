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
TARGET_HOME=$(getent passwd "$TARGET_USER" | cut -d ':' -f 6)
TARGET_GROUP=$(id -gn "$TARGET_USER")

if [[ -z "$TARGET_HOME" || -z "$TARGET_GROUP" ]]; then
  echo "Cannot determine the home directory for user: $TARGET_USER"
  exit 1
fi

# pyclibrary is installed in a local virtual environment: recent Linux distributions mark
# the system Python installation as externally managed (PEP 668) and refuse a plain pip install.
PYCLIBRARY_VENV="$WEBOTS_HOME/dependencies/pyclibrary-venv"
if [ ! -x "$PYCLIBRARY_VENV/bin/python3" ] || ! "$PYCLIBRARY_VENV/bin/python3" -m pip --version > /dev/null 2>&1; then
  rm -rf "$PYCLIBRARY_VENV"
  python3 -m venv "$PYCLIBRARY_VENV"
fi
if ! "$PYCLIBRARY_VENV/bin/python3" -c "import pyclibrary" > /dev/null 2>&1; then
  "$PYCLIBRARY_VENV/bin/python3" -m pip install --no-input pyclibrary
fi
chown -R "$TARGET_USER":"$TARGET_GROUP" "$PYCLIBRARY_VENV"

EMSDK_PATH="$WEBOTS_HOME/dependencies/emsdk"
if [ ! -d "$EMSDK_PATH" ]; then
  git clone https://github.com/emscripten-core/emsdk.git "$EMSDK_PATH"
fi

EMSDK_VERSION_FILE="$EMSDK_PATH/.webots-emsdk-version"
if [ ! -f "$EMSDK_VERSION_FILE" ] || ! grep -qxF "${EMSDK_VERSION}" "$EMSDK_VERSION_FILE"; then
  "$EMSDK_PATH/emsdk" install ${EMSDK_VERSION}
  printf '%s\n' "${EMSDK_VERSION}" > "$EMSDK_VERSION_FILE"
fi
"$EMSDK_PATH/emsdk" activate ${EMSDK_VERSION}
chown -R "$TARGET_USER":"$TARGET_GROUP" "$EMSDK_PATH"

EMSDK_SOURCE_LINE='. "'$EMSDK_PATH'/emsdk_env.sh" >/dev/null 2>&1'
for SHELL_PROFILE in "$TARGET_HOME/.bashrc" "$TARGET_HOME/.bash_profile"; do
  if [ ! -f "$SHELL_PROFILE" ]; then
    runuser -u "$TARGET_USER" -- touch "$SHELL_PROFILE"
  fi
  if ! grep -qxF "$EMSDK_SOURCE_LINE" "$SHELL_PROFILE"; then
    runuser -u "$TARGET_USER" -- sh -c 'printf "%s\n" "$1" >> "$2"' sh "$EMSDK_SOURCE_LINE" "$SHELL_PROFILE"
  fi
  if [ -e "$SHELL_PROFILE" ]; then
    chown "$TARGET_USER":"$TARGET_GROUP" "$SHELL_PROFILE"
  fi
done

if [[ "$OS" == "fedora" ]]; then
    echo "WARNING: Fedora is not an officially supported OS! Dependencies may not be completely installed. Only the two latest Ubuntu LTS are supported."
fi
