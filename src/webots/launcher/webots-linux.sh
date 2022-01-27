#!/bin/bash

# prepare for handling program termination
kill_webots() {
  kill -TERM "${webots_pid}" &> /dev/null
}
handle_termination() {
  if [ "${webots_pid}" ]; then
    kill_webots
  else
    term_kill_needed="yes"
  fi
}
unset webots_pid
unset term_kill_needed
trap 'handle_termination' TERM INT

# get the location of the Webots binary, even if defined into a relative symlinks
webots_home="$(dirname "$(readlink -f "$0")")"

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
  echo "Exec="$webots_home"/webots" >> $FILE
  echo "Icon="$webots_home"/resources/icons/core/webots.png" >> $FILE
  echo "Terminal=false" >> $FILE
  echo "Type=Application" >> $FILE
fi

# we need this to start webots from snap
if [[ ! -z "$SNAP" ]]
then
mkdir -p "$XDG_RUNTIME_DIR"
export QTCOMPOSE=$SNAP/usr/share/X11/locale
fi

if [[ -z "$TMPDIR" ]]
then
TMPDIR=/tmp
fi

if [[ -z "$WEBOTS_TMPDIR" ]]
then
if [[ ! -z "$SNAP" ]]
then
WEBOTS_TMPDIR="$SNAP_USER_COMMON/tmp"
mkdir -p $WEBOTS_TMPDIR
else
WEBOTS_TMPDIR=$TMPDIR
fi
fi

export TMPDIR=$WEBOTS_TMPDIR

# safely create a temporary directory.
# Note that the following two lines cannot be merged into one because `export` would "hide" the return status of `mktemp`.
WEBOTS_TMP_PATH="$(mktemp -d $TMPDIR/webots-$$-XXXXXX)/" || exit 1
export WEBOTS_TMP_PATH

# create temporary lib directory
TMP_LIB_DIR="$WEBOTS_TMP_PATH/lib"
mkdir -p $TMP_LIB_DIR

# add the "lib" directory into LD_LIBRARY_PATH as the first entry
export LD_LIBRARY_PATH="$webots_home/lib/webots":$TMP_LIB_DIR:$LD_LIBRARY_PATH

export QT_ENABLE_HIGHDPI_SCALING=1

# execute the real Webots binary in a child process
if command -v primusrun >/dev/null 2>&1; then
  primusrun "$webots_home/bin/webots-bin" "$@" &
else
  "$webots_home/bin/webots-bin" "$@" &
fi

# wait for termination
webots_pid=$!
if [ "${term_kill_needed}" ]; then
  kill_webots
fi
wait ${webots_pid}
trap - TERM INT
wait ${webots_pid}

webots_return_code=$?

# clean-up tmp folder and pipe files in case webots crashed without clean-up
rm -rf $WEBOTS_TMP_PATH
rm -f ${TMPDIR}/webots_${webots_pid}_*

exit $webots_return_code
