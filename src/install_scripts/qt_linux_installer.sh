#!/bin/bash

# install the latest Qt from qt.org

QT_VERSION=5.11.2
ICU_VERSION=56
QT_INSTALLATION_PATH=~/Qt${QT_VERSION}/${QT_VERSION}/gcc_64
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
rm -rf lib/qt
rm -rf lib/libQt5*
rm -rf lib/libicu*
rm -f resources/web/local/qwebchannel.js

# install Qt in Webots

mkdir include/qt
mkdir include/qt/QtConcurrent
mkdir include/qt/QtCore
mkdir include/qt/QtGui
mkdir include/qt/QtNetwork
mkdir include/qt/QtOpenGL
mkdir include/qt/QtPrintSupport
mkdir include/qt/QtWebChannel
mkdir include/qt/QtWebEngine
mkdir include/qt/QtWebEngineCore
mkdir include/qt/QtWebEngineWidgets
mkdir include/qt/QtWebSockets
mkdir include/qt/QtWidgets
mkdir include/qt/QtXml
mkdir lib/qt
mkdir lib/qt/libexec
mkdir lib/qt/plugins
mkdir lib/qt/plugins/imageformats
mkdir lib/qt/plugins/platforms
mkdir lib/qt/plugins/platformthemes
mkdir lib/qt/plugins/platforminputcontexts
mkdir lib/qt/plugins/printsupport
mkdir lib/qt/plugins/xcbglintegrations
mkdir lib/qt/resources
mkdir lib/qt/translations

cp $QT_INSTALLATION_PATH/bin/lrelease                              bin/qt/
cp $QT_INSTALLATION_PATH/bin/lupdate                               bin/qt/
cp $QT_INSTALLATION_PATH/bin/moc                                   bin/qt/
cp -r $QT_INSTALLATION_PATH/include/QtConcurrent                   include/qt/QtConcurrent/
cp -r $QT_INSTALLATION_PATH/include/QtCore                         include/qt/QtCore/
cp -r $QT_INSTALLATION_PATH/include/QtGui                          include/qt/QtGui/
cp -r $QT_INSTALLATION_PATH/include/QtNetwork                      include/qt/QtNetwork/
cp -r $QT_INSTALLATION_PATH/include/QtOpenGL                       include/qt/QtOpenGL/
cp -r $QT_INSTALLATION_PATH/include/QtPrintSupport                 include/qt/QtPrintSupport/
cp -r $QT_INSTALLATION_PATH/include/QtWebChannel                   include/qt/QtWebChannel/
cp -r $QT_INSTALLATION_PATH/include/QtWebEngine                    include/qt/QtWebEngine/
cp -r $QT_INSTALLATION_PATH/include/QtWebEngineCore                include/qt/QtWebEngineCore/
cp -r $QT_INSTALLATION_PATH/include/QtWebEngineWidgets             include/qt/QtWebEngineWidgets/
cp -r $QT_INSTALLATION_PATH/include/QtWebSockets                   include/qt/QtWebSockets/
cp -r $QT_INSTALLATION_PATH/include/QtWidgets                      include/qt/QtWidgets/
cp -r $QT_INSTALLATION_PATH/include/QtXml                          include/qt/QtXml/
rm -rf include/qt/Qt*/*/$QT_VERSION
cp $QT_INSTALLATION_PATH/lib/libQt5Concurrent.so.$QT_VERSION        lib/
cp $QT_INSTALLATION_PATH/lib/libQt5Core.so.$QT_VERSION              lib/
cp $QT_INSTALLATION_PATH/lib/libQt5DBus.so.$QT_VERSION              lib/
cp $QT_INSTALLATION_PATH/lib/libQt5Gui.so.$QT_VERSION               lib/
cp $QT_INSTALLATION_PATH/lib/libQt5Multimedia.so.$QT_VERSION        lib/
cp $QT_INSTALLATION_PATH/lib/libQt5MultimediaWidgets.so.$QT_VERSION lib/
cp $QT_INSTALLATION_PATH/lib/libQt5Network.so.$QT_VERSION           lib/
cp $QT_INSTALLATION_PATH/lib/libQt5OpenGL.so.$QT_VERSION            lib/
cp $QT_INSTALLATION_PATH/lib/libQt5Positioning.so.$QT_VERSION       lib/
cp $QT_INSTALLATION_PATH/lib/libQt5PrintSupport.so.$QT_VERSION      lib/
cp $QT_INSTALLATION_PATH/lib/libQt5Qml.so.$QT_VERSION               lib/
cp $QT_INSTALLATION_PATH/lib/libQt5Quick.so.$QT_VERSION             lib/
cp $QT_INSTALLATION_PATH/lib/libQt5QuickWidgets.so.$QT_VERSION      lib/
cp $QT_INSTALLATION_PATH/lib/libQt5Sensors.so.$QT_VERSION           lib/
cp $QT_INSTALLATION_PATH/lib/libQt5Sql.so.$QT_VERSION               lib/
cp $QT_INSTALLATION_PATH/lib/libQt5WebChannel.so.$QT_VERSION        lib/
cp $QT_INSTALLATION_PATH/lib/libQt5WebEngine.so.$QT_VERSION         lib/
cp $QT_INSTALLATION_PATH/lib/libQt5WebEngineCore.so.$QT_VERSION     lib/
cp $QT_INSTALLATION_PATH/lib/libQt5WebEngineWidgets.so.$QT_VERSION  lib/
cp $QT_INSTALLATION_PATH/lib/libQt5WebSockets.so.$QT_VERSION        lib/
cp $QT_INSTALLATION_PATH/lib/libQt5Widgets.so.$QT_VERSION           lib/
cp $QT_INSTALLATION_PATH/lib/libQt5XcbQpa.so.$QT_VERSION            lib/
cp $QT_INSTALLATION_PATH/lib/libQt5Xml.so.$QT_VERSION               lib/
cp $QT_INSTALLATION_PATH/lib/libicudata.so.$ICU_VERSION.1           lib/
cp $QT_INSTALLATION_PATH/lib/libicui18n.so.$ICU_VERSION.1           lib/
cp $QT_INSTALLATION_PATH/lib/libicuuc.so.$ICU_VERSION.1             lib/
echo $'[Paths]\nPrefix = ..\n' >                                    lib/qt/libexec/qt.conf
cp $QT_INSTALLATION_PATH/libexec/QtWebEngineProcess                 lib/qt/libexec/
cp $QT_INSTALLATION_PATH/plugins/platforms/libqxcb.so               lib/qt/plugins/platforms/
cp $QT_INSTALLATION_PATH/plugins/platformthemes/libqgtk3.so         lib/qt/plugins/platformthemes/
cp $QT_INSTALLATION_PATH/plugins/platforminputcontexts/libcomposeplatforminputcontextplugin.so lib/qt/plugins/platforminputcontexts/
cp $QT_INSTALLATION_PATH/plugins/platforminputcontexts/libibusplatforminputcontextplugin.so    lib/qt/plugins/platforminputcontexts/
cp $QT_INSTALLATION_PATH/plugins/printsupport/libcupsprintersupport.so                         lib/qt/plugins/printsupport/
cp $QT_INSTALLATION_PATH/plugins/xcbglintegrations/libqxcb-glx-integration.so lib/qt/plugins/xcbglintegrations/
cp $QT_INSTALLATION_PATH/plugins/imageformats/libqjpeg.so           lib/qt/plugins/imageformats/
cp $QT_INSTALLATION_PATH/resources/icudtl.dat                       lib/qt/resources/
cp $QT_INSTALLATION_PATH/resources/qtwebengine_resources.pak        lib/qt/resources/
cp $QT_INSTALLATION_PATH/resources/qtwebengine_resources_100p.pak   lib/qt/resources/
cp $QT_INSTALLATION_PATH/resources/qtwebengine_resources_200p.pak   lib/qt/resources/
cp -r $QT_INSTALLATION_PATH/translations/*                          lib/qt/translations/

# copy qwebchannel.js from the Examples
cp ${QT_INSTALLATION_PATH}/../../Examples/Qt-${QT_VERSION}/webchannel/shared/qwebchannel.js resources/web/local/

cd lib
ln -sf libQt5Concurrent.so.$QT_VERSION        libQt5Concurrent.so.5
ln -sf libQt5Concurrent.so.$QT_VERSION        libQt5Concurrent.so
ln -sf libQt5Core.so.$QT_VERSION              libQt5Core.so.5
ln -sf libQt5Core.so.$QT_VERSION              libQt5Core.so
ln -sf libQt5DBus.so.$QT_VERSION              libQt5DBus.so.5
ln -sf libQt5DBus.so.$QT_VERSION              libQt5DBus.so
ln -sf libQt5Gui.so.$QT_VERSION               libQt5Gui.so.5
ln -sf libQt5Gui.so.$QT_VERSION               libQt5Gui.so
ln -sf libQt5Network.so.$QT_VERSION           libQt5Network.so.5
ln -sf libQt5Network.so.$QT_VERSION           libQt5Network.so
ln -sf libQt5Multimedia.so.$QT_VERSION        libQt5Multimedia.so.5
ln -sf libQt5Multimedia.so.$QT_VERSION        libQt5Multimedia.so
ln -sf libQt5MultimediaWidgets.so.$QT_VERSION libQt5MultimediaWidgets.so.5
ln -sf libQt5MultimediaWidgets.so.$QT_VERSION libQt5MultimediaWidgets.so
ln -sf libQt5OpenGL.so.$QT_VERSION            libQt5OpenGL.so.5
ln -sf libQt5OpenGL.so.$QT_VERSION            libQt5OpenGL.so
ln -sf libQt5Positioning.so.$QT_VERSION       libQt5Positioning.so.5
ln -sf libQt5Positioning.so.$QT_VERSION       libQt5Positioning.so
ln -sf libQt5PrintSupport.so.$QT_VERSION      libQt5PrintSupport.so.5
ln -sf libQt5PrintSupport.so.$QT_VERSION      libQt5PrintSupport.so
ln -sf libQt5Qml.so.$QT_VERSION               libQt5Qml.so.5
ln -sf libQt5Qml.so.$QT_VERSION               libQt5Qml.so
ln -sf libQt5Quick.so.$QT_VERSION             libQt5Quick.so.5
ln -sf libQt5Quick.so.$QT_VERSION             libQt5Quick.so
ln -sf libQt5QuickWidgets.so.$QT_VERSION      libQt5QuickWidgets.so.5
ln -sf libQt5QuickWidgets.so.$QT_VERSION      libQt5QuickWidgets.so
ln -sf libQt5Sensors.so.$QT_VERSION           libQt5Sensors.so.5
ln -sf libQt5Sensors.so.$QT_VERSION           libQt5Sensors.so
ln -sf libQt5Sql.so.$QT_VERSION               libQt5Sql.so.5
ln -sf libQt5Sql.so.$QT_VERSION               libQt5Sql.so
ln -sf libQt5WebChannel.so.$QT_VERSION        libQt5WebChannel.so.5
ln -sf libQt5WebChannel.so.$QT_VERSION        libQt5WebChannel.so
ln -sf libQt5WebEngine.so.$QT_VERSION         libQt5WebEngine.so.5
ln -sf libQt5WebEngine.so.$QT_VERSION         libQt5WebEngine.so
ln -sf libQt5WebEngineCore.so.$QT_VERSION     libQt5WebEngineCore.so.5
ln -sf libQt5WebEngineCore.so.$QT_VERSION     libQt5WebEngineCore.so
ln -sf libQt5WebEngineWidgets.so.$QT_VERSION  libQt5WebEngineWidgets.so.5
ln -sf libQt5WebEngineWidgets.so.$QT_VERSION  libQt5WebEngineWidgets.so
ln -sf libQt5WebSockets.so.$QT_VERSION        libQt5WebSockets.so.5
ln -sf libQt5WebSockets.so.$QT_VERSION        libQt5WebSockets.so
ln -sf libQt5Widgets.so.$QT_VERSION           libQt5Widgets.so.5
ln -sf libQt5Widgets.so.$QT_VERSION           libQt5Widgets.so
ln -sf libQt5Xml.so.$QT_VERSION               libQt5Xml.so.5
ln -sf libQt5Xml.so.$QT_VERSION               libQt5Xml.so
ln -sf libQt5XcbQpa.so.$QT_VERSION            libQt5XcbQpa.so.5
ln -sf libQt5XcbQpa.so.$QT_VERSION            libQt5XcbQpa.so
ln -sf libicudata.so.$ICU_VERSION.1           libicudata.so.$ICU_VERSION
ln -sf libicui18n.so.$ICU_VERSION.1           libicui18n.so.$ICU_VERSION
ln -sf libicuuc.so.$ICU_VERSION.1             libicuuc.so.$ICU_VERSION

cd ..

ARCHIVE=webots-qt-$QT_VERSION-linux64-release.tar.bz2
echo Compressing $ARCHIVE \(please wait\)
tar cjf $ARCHIVE lib/libQt5* lib/libicu* lib/qt include/qt bin/qt/lrelease bin/qt/lupdate bin/qt/moc resources/web/local/qwebchannel.js

echo Done.
