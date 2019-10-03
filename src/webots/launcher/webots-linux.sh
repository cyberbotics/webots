#!/bin/bash
# get the location of the Webots binary, even if defined into a relative symlinks
webotsHome="$(dirname "$(readlink -f "$0")")"

# remove wrong a desktop file if needed
if [ -e ~/.local/share/applications/webots-bin.desktop ]; then
  rm ~/.local/share/applications/webots-bin.desktop
fi

# create a desktop file if it doesn't exist
if [ ! -e /usr/share/applications/webots.desktop ] && [ ! -e ~/.local/share/applications/webots.desktop ]; then
  mkdir -p ~/.local/share/applications
  FILE=~/.local/share/applications/webots.desktop
  echo "[Desktop Entry]" > $FILE
  echo "Name=Webots" >> $FILE
  echo "Comment=Webots mobile robot simulator" >> $FILE
  echo "Exec="$webotsHome"/webots" >> $FILE
  echo "Icon="$webotsHome"/resources/icons/core/webots.png" >> $FILE
  echo "Terminal=false" >> $FILE
  echo "Type=Application" >> $FILE
fi

# we need this to start webots from snap
if [[ ! -z "$SNAP" ]]
then
mkdir -p "$XDG_RUNTIME_DIR"
export QTCOMPOSE=$SNAP/usr/share/X11/locale
fi

# create temporary lib directory
TMP_LIB_DIR="/tmp/webots-$$/lib"
if [ ! -d $TMP_LIB_DIR ]; then
  mkdir -p $TMP_LIB_DIR
fi
export WEBOTS_TMP_PATH="/tmp/webots-$$/"

# add the "lib" directory into LD_LIBRARY_PATH as the first entry
export LD_LIBRARY_PATH="$webotsHome/lib/webots/":$TMP_LIB_DIR:$LD_LIBRARY_PATH

# execute the real Webots binary in a child process
if command -v primusrun >/dev/null 2>&1; then
  primusrun "$webotsHome/bin/webots-bin" "$@" &
else
  "$webotsHome/bin/webots-bin" "$@" &
fi

webots_pid=$!

trap 'kill $webots_pid &> /dev/null' EXIT

wait $webots_pid
webots_return_code=$?

exit $webots_return_code
