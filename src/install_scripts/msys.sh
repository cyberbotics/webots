#!/bin/bash

if [ $# -eq 0 ]
then
export PATH=/mingw64/bin:/usr/bin:/c/WINDOWS/system32
cd ~
else
WINDOWS_DRIVE=${1:1:1}:\\${1:3}
export WEBOTS_HOME=${WINDOWS_DRIVE////\\}
export PATH="$PYTHON37_HOME:$PYTHON37_HOME/Scripts:$1/msys64/mingw64/bin":/mingw64/bin:/usr/bin:$JAVA_HOME/bin:/c/WINDOWS/system32:/c/WINDOWS:$MATLAB_HOME/bin
cd "$1"
fi
