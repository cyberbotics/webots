#!/bin/bash
# Cleanup all Webots preferences.

if [ "$(uname)" == "Darwin" ]; then
  for n in `find $HOME/Library/Preferences -name com.cyberbotics.* | sed 's:.*/::;s:.plist::'`; do
    echo Remove "$n";
    defaults remove $n;
    rm $HOME/Library/Preferences/$n.plist;
  done
elif [[ "$OSTYPE" == "linux-gnu" ]]; then
  for n in `find $HOME/.config/Cyberbotics -name Webots-*.conf`; do
    echo Remove "$n";
    rm $n;
  done
else
  echo "Unsupported OS"
  exit 1
fi
