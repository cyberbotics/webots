#!/bin/bash

# exit when any command fails
set -e

# follow the instructions from https://github.com/cyberbotics/webots/wiki/Qt-compilation#linux to download and compile Qt before executing this script.

QT_VERSION=6.5.3
QT_INSTALLATION_PATH=~/Qt/${QT_VERSION}/gcc_64
WEBOTS_HOME="$(cd "$(dirname "${BASH_SOURCE[0]}" )"/../.. && pwd)"

echo Installing Qt in Webots
echo Source: $QT_INSTALLATION_PATH
echo Destination: $WEBOTS_HOME

# uninstall any previous installation of Qt from Webots

cd $WEBOTS_HOME
rm -f bin/qt/lupdate
rm -f bin/qt/lrelease
rm -f bin/qt/moc
rm -rf include/qt
rm -rf lib/webots/qt
rm -rf lib/webots/libQt6*
rm -rf lib/webots/libicu*

# install Qt in Webots

mkdir include/qt
mkdir include/qt/QtConcurrent
mkdir include/qt/QtCore
mkdir include/qt/QtGui
mkdir include/qt/QtNetwork
mkdir include/qt/QtOpenGL
mkdir include/qt/QtOpenGLWidgets
mkdir include/qt/QtPrintSupport
mkdir include/qt/QtQml
mkdir include/qt/QtWebSockets
mkdir include/qt/QtWidgets
mkdir include/qt/QtXml
mkdir lib/webots/qt
mkdir lib/webots/qt/libexec
mkdir lib/webots/qt/plugins
mkdir lib/webots/qt/plugins/imageformats
mkdir lib/webots/qt/plugins/platforms
mkdir lib/webots/qt/plugins/platformthemes
mkdir lib/webots/qt/plugins/platforminputcontexts
mkdir lib/webots/qt/plugins/printsupport
mkdir lib/webots/qt/plugins/tls
mkdir lib/webots/qt/plugins/xcbglintegrations
mkdir lib/webots/qt/plugins/wayland-graphics-integration-client
mkdir lib/webots/qt/plugins/wayland-shell-integration
mkdir lib/webots/qt/plugins/wayland-decoration-client
mkdir lib/webots/qt/resources
mkdir lib/webots/qt/translations

cp $QT_INSTALLATION_PATH/bin/lrelease                              bin/qt/
cp $QT_INSTALLATION_PATH/bin/lupdate                               bin/qt/
cp $QT_INSTALLATION_PATH/libexec/moc                               bin/qt/
cp -r $QT_INSTALLATION_PATH/include/QtConcurrent                   include/qt/QtConcurrent/
cp -r $QT_INSTALLATION_PATH/include/QtCore                         include/qt/QtCore/
cp -r $QT_INSTALLATION_PATH/include/QtGui                          include/qt/QtGui/
cp -r $QT_INSTALLATION_PATH/include/QtNetwork                      include/qt/QtNetwork/
cp -r $QT_INSTALLATION_PATH/include/QtOpenGL                       include/qt/QtOpenGL/
cp -r $QT_INSTALLATION_PATH/include/QtOpenGLWidgets                include/qt/QtOpenGLWidgets/
cp -r $QT_INSTALLATION_PATH/include/QtPrintSupport                 include/qt/QtPrintSupport/
cp -r $QT_INSTALLATION_PATH/include/QtQml                          include/qt/QtQml/
cp -r $QT_INSTALLATION_PATH/include/QtWebSockets                   include/qt/QtWebSockets/
cp -r $QT_INSTALLATION_PATH/include/QtWidgets                      include/qt/QtWidgets/
cp -r $QT_INSTALLATION_PATH/include/QtXml                          include/qt/QtXml/
rm -rf include/qt/Qt*/*/$QT_VERSION
cp -a $QT_INSTALLATION_PATH/lib/libQt6Concurrent.so*        lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libQt6Core.so*              lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libQt6DBus.so*              lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libQt6Gui.so*               lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libQt6Network.so*           lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libQt6OpenGL.so*            lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libQt6OpenGLWidgets.so*     lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libQt6PrintSupport.so*      lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libQt6Qml.so*               lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libQt6WaylandClient.so*     lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libQt6WaylandEglClientHwIntegration.so*     lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libQt6WebSockets.so*        lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libQt6Widgets.so*           lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libQt6XcbQpa.so*            lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libQt6Xml.so*               lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libicudata.so*              lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libicui18n.so*              lib/webots/
cp -a $QT_INSTALLATION_PATH/lib/libicuuc.so*                lib/webots/
echo $'[Paths]\nPrefix = ..\n' >                                    lib/webots/qt/libexec/qt.conf
cp -a $QT_INSTALLATION_PATH/plugins/platforms/libqxcb.so               lib/webots/qt/plugins/platforms/
cp -a $QT_INSTALLATION_PATH/plugins/platforms/libqwayland-egl.so       lib/webots/qt/plugins/platforms/
cp -a $QT_INSTALLATION_PATH/plugins/platforms/libqwayland-generic.so   lib/webots/qt/plugins/platforms/
cp -a $QT_INSTALLATION_PATH/plugins/platformthemes/*         lib/webots/qt/plugins/platformthemes/
cp -a $QT_INSTALLATION_PATH/plugins/platforminputcontexts/libcomposeplatforminputcontextplugin.so lib/webots/qt/plugins/platforminputcontexts/
cp -a $QT_INSTALLATION_PATH/plugins/platforminputcontexts/libibusplatforminputcontextplugin.so    lib/webots/qt/plugins/platforminputcontexts/
cp -a $QT_INSTALLATION_PATH/plugins/printsupport/libcupsprintersupport.so lib/webots/qt/plugins/printsupport/
cp -a $QT_INSTALLATION_PATH/plugins/tls/*.so                           lib/webots/qt/plugins/tls/
cp -a $QT_INSTALLATION_PATH/plugins/xcbglintegrations/libqxcb-glx-integration.so lib/webots/qt/plugins/xcbglintegrations/
cp -a $QT_INSTALLATION_PATH/plugins/wayland-graphics-integration-client/* lib/webots/qt/plugins/wayland-graphics-integration-client/
cp -a $QT_INSTALLATION_PATH/plugins/wayland-shell-integration/*        lib/webots/qt/plugins/wayland-shell-integration/
cp -a $QT_INSTALLATION_PATH/plugins/wayland-decoration-client/*        lib/webots/qt/plugins/wayland-decoration-client/
cp -a $QT_INSTALLATION_PATH/plugins/imageformats/libqjpeg.so           lib/webots/qt/plugins/imageformats/
cp -a $QT_INSTALLATION_PATH/translations/qt_*                       lib/webots/qt/translations/
cp -a $QT_INSTALLATION_PATH/translations/qtbase_*                   lib/webots/qt/translations/
cp -a $QT_INSTALLATION_PATH/translations/qtdeclarative_*            lib/webots/qt/translations/
cp -a $QT_INSTALLATION_PATH/translations/qtwebsockets_*             lib/webots/qt/translations/

ARCHIVE=dependencies/webots-qt-$QT_VERSION-linux64-release.tar.bz2
echo Compressing $ARCHIVE \(please wait\)
tar cjf $ARCHIVE lib/webots/libQt6* lib/webots/libicu* lib/webots/qt include/qt bin/qt/lrelease bin/qt/lupdate bin/qt/moc

echo Done.
