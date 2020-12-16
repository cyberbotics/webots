#!/usr/bin/env bash

# This script lists the DLLs needed by ffmpeg that we not already needed by Webots, to be added to the distribution

# We first test if ffmpeg can be executed and fail otherwise
set -e
ffmpeg.exe -version > /dev/null

# We generate the list of DLL dependencies for ffmpeg.exe that are not already dependencies of webots-bin.exe
ldd /mingw64/bin/ffmpeg.exe | grep /mingw64/bin | awk '{print $1;}' | sort -u > ffmpeg.tmp
ldd ${WEBOTS_HOME}/msys64/mingw64/bin/webots-bin.exe | grep /mingw64/bin | awk '{print $1;}' | sort -u > webots.tmp
comm -23 ffmpeg.tmp webots.tmp
rm ffmpeg.tmp webots.tmp
