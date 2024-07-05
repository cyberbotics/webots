#!/bin/bash

# exit when any command fails
set -e

# Usage: msys64_installer.sh [--all]
# Options:
# --all: install all the optional dependencies

declare -a BASE_PACKAGES=(
  "make"                              # Makefile
  "mingw-w64-x86_64-gcc"              # C/C++ compiler
  "tar"                               # Webots dependencies
  "unzip"                             # Webots dependencies
  "zip"                               # robotbenchmark square path
  "mingw-w64-x86_64-qt6-base"         # Webots
  "mingw-w64-x86_64-qt6-declarative"  # Webots
  "mingw-w64-x86_64-qt6-tools"        # Webots (translation: lrelease-qt6 and lupdate-qt6)
  "mingw-w64-x86_64-qt6-translations" # Webots
  "mingw-w64-x86_64-qt6-websockets"   # Webots
  "mingw-w64-x86_64-libzip"           # Webots
  "mingw-w64-x86_64-woff2"            # Webots
  "mingw-w64-x86_64-minizip"          # Webots (assimp)
  "mingw-w64-x86_64-zlib"             # Webots (assimp)
  "liblzma"                           # Webots
  "mingw-w64-x86_64-ffmpeg"           # Webots movies
  "mingw-w64-x86_64-dlfcn"            # dependency of ffmpeg
  "mingw-w64-x86_64-python"           # python controllers and scripts
)

declare -a OPTIONAL_PACKAGES=(
  "git"                         # Distribution script (check_submodules_update.sh)
  "pacman-contrib"              # Distribution script (pactree)
  "swig"                        # Python and Java API wrappers
  "mingw-w64-x86_64-boost"      # to recompile ROS controller
  "mingw-w64-x86_64-python-pip" # Useful for advanced python usage
)

declare -a DEVELOPMENT_PACKAGES=(
  "mingw-w64-x86_64-clang"    # coding style tests
  "mingw-w64-x86_64-cppcheck" # coding style tests
  "mingw-w64-x86_64-gdb"      # debugging
  "diffutils"                 # cmp and diff utilities
)

if [ "$1" == "--dev" ]; then
  declare -a PACKAGES=("${BASE_PACKAGES[@]}" "${OPTIONAL_PACKAGES[@]}" "${DEVELOPMENT_PACKAGES[@]}")
elif [ "$1" == "--all" ]; then
  declare -a PACKAGES=("${BASE_PACKAGES[@]}" "${OPTIONAL_PACKAGES[@]}")
else
  declare -a PACKAGES=("${BASE_PACKAGES[@]}")
fi

for i in "${PACKAGES[@]}"
do
  if ! pacman -Q $i 2> /dev/null
  then
    echo Installing $i
    pacman -S --noconfirm $i
    pacman -Scc --noconfirm
  else
    echo Skipping $i \(already installed\)
  fi
done

script_name=$0
script_full_path=$(dirname "$0")
$script_full_path/qt_windows_installer.sh
