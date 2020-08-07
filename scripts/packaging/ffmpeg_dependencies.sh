#!/usr/bin/env bash

# This script lists the DLLs needed by ffmpeg that we not already needed by Webots, to be added to the distribution
ldd /mingw64/bin/ffmpeg.exe | grep /mingw64/bin | awk '{print $1;}' | sort -u > ffmpeg.tmp
ldd ${WEBOTS_HOME}/msys64/mingw64/bin/webots-bin.exe | grep /mingw64/bin | awk '{print $1;}' | sort -u > webots.tmp
comm -23 ffmpeg.tmp webots.tmp
rm ffmpeg.tmp webots.tmp
