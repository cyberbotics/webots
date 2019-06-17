#!/usr/bin/env bash

ldd /mingw64/bin/ffmpeg.exe | grep /mingw64/bin | awk '{print $1;}' | sort -u > ffmpeg.tmp
ldd ../../msys64/mingw64/bin/webots.exe | grep /mingw64/bin | awk '{print $1;}' | sort -u > webots.tmp
comm -23 ffmpeg.tmp webots.tmp
rm ffmpeg.tmp webots.tmp
