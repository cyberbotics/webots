#!/bin/bash

# exit when any command fails
set -e

QT_VERSION=6.5.3
pip install --no-input aqtinstall
aqt install-qt --outputdir ~/Qt linux desktop ${QT_VERSION} gcc_64 -m qtwebsockets
QT_INSTALLATION_PATH=~/Qt/${QT_VERSION}/gcc_64
QT_INSTALLATION_BIN_PATH=${QT_INSTALLATION_PATH}/bin
QT_INSTALLATION_LIBEXEC_PATH=${QT_INSTALLATION_PATH}/libexec
QT_INSTALLATION_LIB_PATH=${QT_INSTALLATION_PATH}/lib
QT_INSTALLATION_INCLUDE_PATH=${QT_INSTALLATION_PATH}/include
QT_INSTALLATION_PLUGINS_PATH=${QT_INSTALLATION_PATH}/plugins
QT_INSTALLATION_TRANSLATIONS_PATH=${QT_INSTALLATION_PATH}/translations

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

cp $QT_INSTALLATION_BIN_PATH/lrelease                              bin/qt/
cp $QT_INSTALLATION_BIN_PATH/lupdate                               bin/qt/
cp $QT_INSTALLATION_LIBEXEC_PATH/moc                               bin/qt/
cp -r $QT_INSTALLATION_INCLUDE_PATH/QtConcurrent                   include/qt/QtConcurrent/
cp -r $QT_INSTALLATION_INCLUDE_PATH/QtCore                         include/qt/QtCore/
cp -r $QT_INSTALLATION_INCLUDE_PATH/QtGui                          include/qt/QtGui/
cp -r $QT_INSTALLATION_INCLUDE_PATH/QtNetwork                      include/qt/QtNetwork/
cp -r $QT_INSTALLATION_INCLUDE_PATH/QtOpenGL                       include/qt/QtOpenGL/
cp -r $QT_INSTALLATION_INCLUDE_PATH/QtOpenGLWidgets                include/qt/QtOpenGLWidgets/
cp -r $QT_INSTALLATION_INCLUDE_PATH/QtPrintSupport                 include/qt/QtPrintSupport/
cp -r $QT_INSTALLATION_INCLUDE_PATH/QtQml                          include/qt/QtQml/
cp -r $QT_INSTALLATION_INCLUDE_PATH/QtWebSockets                   include/qt/QtWebSockets/
cp -r $QT_INSTALLATION_INCLUDE_PATH/QtWidgets                      include/qt/QtWidgets/
cp -r $QT_INSTALLATION_INCLUDE_PATH/QtXml                          include/qt/QtXml/
rm -rf include/qt/Qt*/*/$QT_VERSION
cp -a $QT_INSTALLATION_LIB_PATH/libQt6Concurrent.so*        lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libQt6Core.so*              lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libQt6DBus.so*              lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libQt6Gui.so*               lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libQt6Network.so*           lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libQt6OpenGL.so*            lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libQt6OpenGLWidgets.so*     lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libQt6PrintSupport.so*      lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libQt6Qml.so*               lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libQt6WaylandClient.so*     lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libQt6WaylandEglClientHwIntegration.so*     lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libQt6WebSockets.so*        lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libQt6Widgets.so*           lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libQt6XcbQpa.so*            lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libQt6Xml.so*               lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libicudata.so*              lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libicui18n.so*              lib/webots/
cp -a $QT_INSTALLATION_LIB_PATH/libicuuc.so*                lib/webots/
echo $'[Paths]\nPrefix = ..\n' >                                    lib/webots/qt/libexec/qt.conf
cp -a $QT_INSTALLATION_PLUGINS_PATH/platforms/libqxcb.so               lib/webots/qt/plugins/platforms/
cp -a $QT_INSTALLATION_PLUGINS_PATH/platforms/libqwayland-egl.so       lib/webots/qt/plugins/platforms/
cp -a $QT_INSTALLATION_PLUGINS_PATH/platforms/libqwayland-generic.so   lib/webots/qt/plugins/platforms/
cp -a $QT_INSTALLATION_PLUGINS_PATH/platformthemes/*         lib/webots/qt/plugins/platformthemes/
cp -a $QT_INSTALLATION_PLUGINS_PATH/platforminputcontexts/libcomposeplatforminputcontextplugin.so lib/webots/qt/plugins/platforminputcontexts/
cp -a $QT_INSTALLATION_PLUGINS_PATH/platforminputcontexts/libibusplatforminputcontextplugin.so    lib/webots/qt/plugins/platforminputcontexts/
cp -a $QT_INSTALLATION_PLUGINS_PATH/printsupport/libcupsprintersupport.so lib/webots/qt/plugins/printsupport/
cp -a $QT_INSTALLATION_PLUGINS_PATH/tls/*.so                           lib/webots/qt/plugins/tls/
cp -a $QT_INSTALLATION_PLUGINS_PATH/xcbglintegrations/libqxcb-glx-integration.so lib/webots/qt/plugins/xcbglintegrations/
cp -a $QT_INSTALLATION_PLUGINS_PATH/wayland-graphics-integration-client/* lib/webots/qt/plugins/wayland-graphics-integration-client/
cp -a $QT_INSTALLATION_PLUGINS_PATH/wayland-shell-integration/*        lib/webots/qt/plugins/wayland-shell-integration/
cp -a $QT_INSTALLATION_PLUGINS_PATH/wayland-decoration-client/*        lib/webots/qt/plugins/wayland-decoration-client/
cp -a $QT_INSTALLATION_PLUGINS_PATH/imageformats/libqjpeg.so           lib/webots/qt/plugins/imageformats/
cp -a $QT_INSTALLATION_TRANSLATIONS_PATH/qt_*                       lib/webots/qt/translations/
cp -a $QT_INSTALLATION_TRANSLATIONS_PATH/qtbase_*                   lib/webots/qt/translations/
cp -a $QT_INSTALLATION_TRANSLATIONS_PATH/qtdeclarative_*            lib/webots/qt/translations/
cp -a $QT_INSTALLATION_TRANSLATIONS_PATH/qtwebsockets_*             lib/webots/qt/translations/

ARCHIVE=dependencies/webots-qt-$QT_VERSION-linux64-release.tar.bz2
echo Compressing $ARCHIVE \(please wait\)
tar cjf $ARCHIVE lib/webots/libQt6* lib/webots/libicu* lib/webots/qt include/qt bin/qt/lrelease bin/qt/lupdate bin/qt/moc

echo Done.
