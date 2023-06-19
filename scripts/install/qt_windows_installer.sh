#!/bin/bash

# exit when any command fails
set -e

# install the latest Qt6 which comes with msys64

QT_BASE_PACKAGE=mingw-w64-x86_64-qt6-base

PACMAN_AVAILABLE_QT_VERSION=(`pacman -Syi $QT_BASE_PACKAGE | grep Version`)
IFS='-' read -a QT_AVAILABLE_VERSION_SPLIT <<< "${PACMAN_AVAILABLE_QT_VERSION[2]}"
AVAILABLE_QT_VERSION=${QT_AVAILABLE_VERSION_SPLIT[0]}

PACMAN_INSTALLED_QT_VERSION=(`pacman -Qi $QT_BASE_PACKAGE | grep Version`)
IFS='-' read -a QT_INSTALLED_VERSION_SPLIT <<< "${PACMAN_AVAILABLE_QT_VERSION[2]}"
INSTALLED_QT_VERSION=${QT_INSTALLED_VERSION_SPLIT[0]}

REQUESTED_QT_VERSION=$AVAILABLE_QT_VERSION  # we usually need to use the latest version of Qt6

echo Qt6 version requested: $REQUESTED_QT_VERSION
echo Qt6 version installed: $INSTALLED_QT_VERSION
echo Qt6 version available: $AVAILABLE_QT_VERSION

if [[ $REQUESTED_QT_VERSION != $INSTALLED_QT_VERSION ]] && [[ $REQUESTED_QT_VERSION != $AVAILABLE_QT_VERSION ]]
then
  echo Requested version not available
  exit
fi

if [[ $PACMAN_AVAILABLE_QT_VERSION != $PACMAN_INSTALLED_QT_VERSION ]]
then
  echo Installing Qt6 version ${PACMAN_AVAILABLE_QT_VERSION[2]}
  pacman -Sy --noconfirm $QT_BASE_PACKAGE
  pacman -Sy --noconfirm mingw-w64-x86_64-qt6-declarative
  pacman -Sy --noconfirm mingw-w64-x86_64-qt6-tools
  pacman -Sy --noconfirm mingw-w64-x86_64-qt6-translations
  pacman -Sy --noconfirm mingw-w64-x86_64-qt6-websockets
fi

script_name=$0
script_full_path=$(dirname "$0")
cd $script_full_path/../..

rm -rf include/qt

for MODULE in QtConcurrent QtCore QtGui QtNetwork QtOpenGL QtOpenGLWidgets QtPrintSupport QtQml QtWebSockets QtWidgets QtXml
do
  echo installing $MODULE...
  mkdir -p include/qt/$MODULE/$MODULE
  # Ignore errors while copying
  cp /mingw64/include/qt6/$MODULE/* include/qt/$MODULE/$MODULE/ 2>&1 | grep -v 'omitting directory' || true
done

echo done.
