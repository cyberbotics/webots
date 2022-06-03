#!/bin/bash
# get the location of the Webots binary, even if defined into a relative symlink
webotsBinaryFullPath="$0"
while [ -h "$webotsBinaryFullPath" ] ; do
  webotsBinaryFullPath=$(readlink "$webotsBinaryFullPath")
done
webotsBinaryDir=$(dirname "$webotsBinaryFullPath")

# execute the real Webots binary in a child process
"$webotsBinaryDir"/Contents/MacOS/webots "$@" &

webots_pid=$!

trap 'kill $webots_pid &> /dev/null' EXIT

wait $webots_pid
