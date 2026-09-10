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

refresh_emsdk_clone() {
  if [ -d "$EMSDK_PATH/.git" ]; then
    git -C "$EMSDK_PATH" pull --ff-only
  else
    rm -rf "$EMSDK_PATH"
    git clone https://github.com/emscripten-core/emsdk.git "$EMSDK_PATH"
  fi
}

emsdk_has_version_installed() {
  "$EMSDK_PATH/emsdk" list --installed | grep -qw "${EMSDK_VERSION}"
}

emsdk_can_resolve_version() {
  "$EMSDK_PATH/emsdk" list | grep -qw "${EMSDK_VERSION}"
}

emsdk_is_version_active() {
  python3 - "$EMSDK_PATH" "$EMSDK_VERSION" <<'PY'
import importlib.util
import os
import pathlib
import sys

emsdk_path = pathlib.Path(sys.argv[1]).resolve()
requested_version = sys.argv[2]
os.chdir(emsdk_path)
spec = importlib.util.spec_from_file_location('webots_emsdk', emsdk_path / 'emsdk.py')
emsdk = importlib.util.module_from_spec(spec)
spec.loader.exec_module(emsdk)
active_sdk = emsdk.currently_active_sdk()
resolved_version = emsdk.resolve_sdk_aliases(requested_version)
sys.exit(0 if active_sdk and str(active_sdk) == resolved_version else 1)
PY
}

EMSDK_VERSION_FILE="$EMSDK_PATH/.webots-emsdk-version"
EMSDK_BINARY="$EMSDK_PATH/upstream/emscripten/emcc"
NEEDS_EMSDK_INSTALL=true
if [ -x "$EMSDK_BINARY" ] && [ -f "$EMSDK_VERSION_FILE" ] && grep -qxF "${EMSDK_VERSION}" "$EMSDK_VERSION_FILE" && \
   emsdk_has_version_installed; then
  NEEDS_EMSDK_INSTALL=false
fi
if [[ "$NEEDS_EMSDK_INSTALL" == true ]]; then
  refresh_emsdk_clone
  if ! emsdk_can_resolve_version; then
    rm -rf "$EMSDK_PATH"
    git clone https://github.com/emscripten-core/emsdk.git "$EMSDK_PATH"
    if ! emsdk_can_resolve_version; then
      echo "Cannot resolve EMSDK ${EMSDK_VERSION}"
      exit 1
    fi
  fi
  "$EMSDK_PATH/emsdk" install "${EMSDK_VERSION}"
fi
if ! "$EMSDK_PATH/emsdk" activate "${EMSDK_VERSION}"; then
  echo "Failed to activate EMSDK ${EMSDK_VERSION}"
  exit 1
fi
if ! emsdk_is_version_active; then
  echo "Failed to activate EMSDK ${EMSDK_VERSION}"
  exit 1
fi
if [ ! -x "$EMSDK_BINARY" ] || ! emsdk_has_version_installed; then
  echo "Failed to install EMSDK ${EMSDK_VERSION}"
  exit 1
fi
chown -R "$TARGET_USER":"$TARGET_GROUP" "$EMSDK_PATH"
if ! runuser -u "$TARGET_USER" -- test -r "$EMSDK_PATH/emsdk_env.sh"; then
  echo "Cannot access $EMSDK_PATH/emsdk_env.sh as $TARGET_USER"
  exit 1
fi
if [ -f "$EMSDK_VERSION_FILE" ]; then
  chown "$TARGET_USER":"$TARGET_GROUP" "$EMSDK_VERSION_FILE"
else
  runuser -u "$TARGET_USER" -- touch "$EMSDK_VERSION_FILE"
fi
runuser -u "$TARGET_USER" -- sh -c 'printf "%s\n" "$1" > "$2"' sh "${EMSDK_VERSION}" "$EMSDK_VERSION_FILE"

EMSDK_SOURCE_LINE='. "'$EMSDK_PATH'/emsdk_env.sh" >/dev/null 2>&1'
EMSDK_LEGACY_SOURCE_LINE='source "'$EMSDK_PATH'/emsdk_env.sh" >/dev/null 2>&1'
BASHRC_PROFILE="$TARGET_HOME/.bashrc"

BASH_PROFILE=
BASH_PROFILE_CANDIDATE="$TARGET_HOME/.bash_profile"
BASH_LOGIN="$TARGET_HOME/.bash_login"
BASH_DOT_PROFILE="$TARGET_HOME/.profile"
BASH_LOGIN_PROFILES=("$BASH_PROFILE_CANDIDATE" "$BASH_LOGIN" "$BASH_DOT_PROFILE")
BASH_PROFILE="$BASH_DOT_PROFILE"
BASHRC_SOURCE_LINE='if [ -f "$HOME/.bashrc" ]; then . "$HOME/.bashrc"; fi'
for profile in "${BASH_LOGIN_PROFILES[@]}"; do
  if [ -f "$profile" ]; then
    BASH_PROFILE="$profile"
    break
  fi
done
if [ ! -f "$BASH_PROFILE" ]; then
  runuser -u "$TARGET_USER" -- touch "$BASH_PROFILE"
fi
for profile in "${BASH_LOGIN_PROFILES[@]}"; do
  if [ -f "$profile" ]; then
    runuser -u "$TARGET_USER" -- env EMSDK_SOURCE_LINE="$EMSDK_SOURCE_LINE" EMSDK_LEGACY_SOURCE_LINE="$EMSDK_LEGACY_SOURCE_LINE" \
      python3 -c 'import os, pathlib, sys
path = pathlib.Path(sys.argv[1])
skip = {os.environ["EMSDK_SOURCE_LINE"], os.environ["EMSDK_LEGACY_SOURCE_LINE"]}
lines = path.read_text().splitlines()
path.write_text("".join(f"{line}\n" for line in lines if line not in skip))' "$profile"
  fi
done
for profile in "${BASH_LOGIN_PROFILES[@]}"; do
  if [ -f "$profile" ] && ! grep -qxF "$BASHRC_SOURCE_LINE" "$profile"; then
    runuser -u "$TARGET_USER" -- sh -c 'printf "%s\n" "$1" >> "$2"' sh "$BASHRC_SOURCE_LINE" "$profile"
  fi
done
chown "$TARGET_USER":"$TARGET_GROUP" "$BASH_PROFILE"

if [ ! -f "$BASHRC_PROFILE" ]; then
  runuser -u "$TARGET_USER" -- touch "$BASHRC_PROFILE"
fi
if ! grep -qxF "$EMSDK_SOURCE_LINE" "$BASHRC_PROFILE"; then
  runuser -u "$TARGET_USER" -- sh -c 'printf "%s\n" "$1" >> "$2"' sh "$EMSDK_SOURCE_LINE" "$BASHRC_PROFILE"
fi
chown "$TARGET_USER":"$TARGET_GROUP" "$BASHRC_PROFILE"

if [[ "$OS" == "fedora" ]]; then
    echo "WARNING: Fedora is not an officially supported OS! Dependencies may not be completely installed. Only the two latest Ubuntu LTS are supported."
fi
