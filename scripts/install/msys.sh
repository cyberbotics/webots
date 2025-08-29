#!/bin/bash

if [ $# -eq 0 ]; then
export PATH=/mingw64/bin:/usr/bin:/c/WINDOWS/system32
cd ~
else
WINDOWS_DRIVE=${1:1:1}:\\${1:3}
export WEBOTS_HOME=${WINDOWS_DRIVE////\\}
PATH="$1/msys64/mingw64/bin"
if [ -v PYTHON_HOME ]; then
PATH+=":$PYTHON_HOME:$PYTHON_HOME/Scripts"
fi
PATH+=":/mingw64/bin:/usr/bin"
if [ -v JAVA_HOME ]; then
PATH+=":$JAVA_HOME/bin"
fi
PATH+=":/c/WINDOWS/system32:/c/WINDOWS"
if [ -v MATLAB_HOME ]; then
PATH+=":$MATLAB_HOME/bin"
fi
export PATH=$PATH
cd ~
if [[ "$1" == *"$MSYS64_HOME"/home/"$USER/"* ]]; then
cd ${1#$MSYS64_HOME"/home/"$USER/}
fi
fi
