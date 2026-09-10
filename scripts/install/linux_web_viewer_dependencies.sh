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
EMSDK_BINARY="$EMSDK_PATH/upstream/emscripten/emcc"
NEEDS_EMSDK_INSTALL=true
if [ -x "$EMSDK_BINARY" ] && [ -f "$EMSDK_VERSION_FILE" ] && grep -qxF "${EMSDK_VERSION}" "$EMSDK_VERSION_FILE"; then
  NEEDS_EMSDK_INSTALL=false
fi
if [[ "$NEEDS_EMSDK_INSTALL" == true ]]; then
  "$EMSDK_PATH/emsdk" install ${EMSDK_VERSION}
fi
"$EMSDK_PATH/emsdk" activate ${EMSDK_VERSION}
printf '%s\n' "${EMSDK_VERSION}" > "$EMSDK_VERSION_FILE"
chown -R "$TARGET_USER":"$TARGET_GROUP" "$EMSDK_PATH"

EMSDK_SOURCE_LINE='. "'$EMSDK_PATH'/emsdk_env.sh" >/dev/null 2>&1'
EMSDK_LEGACY_SOURCE_LINE='source "'$EMSDK_PATH'/emsdk_env.sh" >/dev/null 2>&1'
BASHRC_PROFILE="$TARGET_HOME/.bashrc"
if [ ! -f "$BASHRC_PROFILE" ]; then
  runuser -u "$TARGET_USER" -- touch "$BASHRC_PROFILE"
fi
if ! grep -qxF "$EMSDK_SOURCE_LINE" "$BASHRC_PROFILE"; then
  runuser -u "$TARGET_USER" -- sh -c 'printf "%s\n" "$1" >> "$2"' sh "$EMSDK_SOURCE_LINE" "$BASHRC_PROFILE"
fi
chown "$TARGET_USER":"$TARGET_GROUP" "$BASHRC_PROFILE"

BASH_PROFILE="$TARGET_HOME/.bash_profile"
BASHRC_SOURCE_LINE='if [ -f "$HOME/.bashrc" ]; then . "$HOME/.bashrc"; fi'
if [ ! -f "$BASH_PROFILE" ]; then
  runuser -u "$TARGET_USER" -- touch "$BASH_PROFILE"
fi
runuser -u "$TARGET_USER" -- python3 - "$BASH_PROFILE" "$EMSDK_SOURCE_LINE" "$EMSDK_LEGACY_SOURCE_LINE" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
lines_to_remove = set(sys.argv[2:])
lines = path.read_text().splitlines()
path.write_text(''.join(f'{line}\n' for line in lines if line not in lines_to_remove))
PY
if ! grep -qxF "$BASHRC_SOURCE_LINE" "$BASH_PROFILE"; then
  runuser -u "$TARGET_USER" -- sh -c 'printf "%s\n" "$1" >> "$2"' sh "$BASHRC_SOURCE_LINE" "$BASH_PROFILE"
fi
chown "$TARGET_USER":"$TARGET_GROUP" "$BASH_PROFILE"

if [[ "$OS" == "fedora" ]]; then
    echo "WARNING: Fedora is not an officially supported OS! Dependencies may not be completely installed. Only the two latest Ubuntu LTS are supported."
fi
