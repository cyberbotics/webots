#!/usr/bin/env bash

ldd /mingw64/bin/ffmpeg.exe | grep /mingw64/bin | awk '{print $1;}' | sort -u > ffmpeg.txt
ldd ../../msys64/mingw64/bin/webots.exe | grep /mingw64/bin | awk '{print $1;}' | sort -u > webots.txt
comm -23 ffmpeg.txt webots.txt > ffmpeg_dependencies.txt
rm ffmpeg.txt webots.txt
