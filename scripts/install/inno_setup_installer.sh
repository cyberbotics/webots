#!/usr/bin/env bash

if [ ! -f ./is.exe ]; then
  wget -q http://www.jrsoftware.org/download.php/is.exe
fi
./is.exe //VERYSILENT //SUPRESSMSGBOXES //ALLUSERS
rm is.exe
