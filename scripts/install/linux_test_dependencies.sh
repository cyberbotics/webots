#!/bin/bash

# exit when any command fails on CI
if [[ ! -z "$CI" ]]; then
       set -e
fi

if [[ $EUID -ne 0 ]]; then
       echo "This script must be run as root"
       exit 1
fi

alias apt='apt --option="APT::Acquire::Retries=3"'
apt install liburdfdom-tools -y

if [[ $@ != *"--norecurse"* ]]; then
  echo "Installing runtime dependencies"
  script_full_path=$(dirname "$0")
  $script_full_path/linux_runtime_dependencies.sh
fi
