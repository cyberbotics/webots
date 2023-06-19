#!/bin/bash

# exit when any command fails
set -e

if [[ -z "${WEBOTS_HOME}" ]]; then
  echo "WEBOTS_HOME is not defined."
  exit 1
fi

QT_VERSION=6.4.3

# prepare Webots
cd $WEBOTS_HOME
rm -fr Contents/Frameworks/Qt* bin/qt/lupdate bin/qt/lrelease bin/qt/moc include/qt lib/webots/qt
mkdir lib/webots/qt
mkdir include/qt

# populate webots
cd $HOME/Qt/$QT_VERSION/macos
cp bin/lrelease $WEBOTS_HOME/bin/qt
cp bin/lupdate $WEBOTS_HOME/bin/qt
cp libexec/moc $WEBOTS_HOME/bin/qt

declare -a qtFrameworks=( \
  "QtConcurrent" \
  "QtCore" \
  "QtDBus" \
  "QtGui" \
  "QtNetwork" \
  "QtOpenGL" \
  "QtOpenGLWidgets" \
  "QtPrintSupport" \
  "QtQml" \
  "QtWebSockets" \
  "QtWidgets" \
  "QtXml" \
)

for f in "${qtFrameworks[@]}"
do
  cp -R lib/$f.framework $WEBOTS_HOME/Contents/Frameworks
done

mkdir $WEBOTS_HOME/lib/webots/qt/plugins
mkdir $WEBOTS_HOME/lib/webots/qt/plugins/imageformats
mkdir $WEBOTS_HOME/lib/webots/qt/plugins/platforms
mkdir $WEBOTS_HOME/lib/webots/qt/plugins/styles
mkdir $WEBOTS_HOME/lib/webots/qt/plugins/tls
mkdir $WEBOTS_HOME/lib/webots/qt/libexec
cp plugins/imageformats/libqjpeg.dylib $WEBOTS_HOME/lib/webots/qt/plugins/imageformats/
cp plugins/platforms/libqcocoa.dylib $WEBOTS_HOME/lib/webots/qt/plugins/platforms/
cp plugins/styles/libqmacstyle.dylib $WEBOTS_HOME/lib/webots/qt/plugins/styles/
cp plugins/tls/*.dylib $WEBOTS_HOME/lib/webots/qt/plugins/tls/
echo $'[Paths]\nPrefix = ..\n' > $WEBOTS_HOME/lib/webots/qt/libexec/qt.conf

# remove the debug frameworks
cd  $WEBOTS_HOME/Contents/Frameworks/
find . -name *_debug | xargs rm

# Render the frameworks relative to the executable:
cd  $WEBOTS_HOME/Contents/Frameworks/

for fA in "${qtFrameworks[@]}"
do
   install_name_tool -id @rpath/Contents/Frameworks/$fA.framework/Versions/A/$fA $fA.framework/Versions/A/$fA
   for fB in "${qtFrameworks[@]}"
   do
     install_name_tool -change @rpath/$fB.framework/Versions/A/$fB @rpath/Contents/Frameworks/$fB.framework/Versions/A/$fB $fA.framework/Versions/A/$fA
   done
done

# Render the plugins relative to the executable:
cd $WEBOTS_HOME/lib/webots/qt/plugins

declare -a libs=("imageformats/libqjpeg.dylib" "platforms/libqcocoa.dylib" "styles/libqmacstyle.dylib" "tls/libqsecuretransportbackend.dylib" "tls/libqcertonlybackend.dylib" "tls/libqopensslbackend.dylib")

for lib in "${libs[@]}"
do
  install_name_tool -id @rpath/lib/webots/qt/plugins/$lib $lib
  install_name_tool -add_rpath @loader_path/../../../.. $lib
  for f in "${qtFrameworks[@]}"
  do
    install_name_tool -change @rpath/$f.framework/Versions/A/$f @rpath/Contents/Frameworks/$f.framework/Versions/A/$f $lib
  done
done

# Change the RPATH of the executables
cd $WEBOTS_HOME/bin/qt
install_name_tool -rpath @loader_path/../lib @loader_path/../.. lrelease
install_name_tool -change @rpath/QtCore.framework/Versions/A/QtCore @rpath/Contents/Frameworks/QtCore.framework/Versions/A/QtCore lrelease
install_name_tool -change @rpath/QtXml.framework/Versions/A/QtXml @rpath/Contents/Frameworks/QtXml.framework/Versions/A/QtXml lrelease
install_name_tool -rpath @loader_path/../lib @loader_path/../.. lupdate
install_name_tool -change @rpath/QtCore.framework/Versions/A/QtCore @rpath/Contents/Frameworks/QtCore.framework/Versions/A/QtCore lupdate
install_name_tool -change @rpath/QtXml.framework/Versions/A/QtXml @rpath/Contents/Frameworks/QtXml.framework/Versions/A/QtXml lupdate
install_name_tool -change @rpath/QtQml.framework/Versions/A/QtQml @rpath/Contents/Frameworks/QtQml.framework/Versions/A/QtQml lupdate
install_name_tool -change @rpath/QtNetwork.framework/Versions/A/QtNetwork @rpath/Contents/Frameworks/QtNetwork.framework/Versions/A/QtNetwork lupdate
install_name_tool -add_rpath @loader_path/../.. moc

cd $WEBOTS_HOME

ARCHIVE=qt-$QT_VERSION-release.tar.bz2
echo Compressing $ARCHIVE \(please wait\)
tar cjf $ARCHIVE Contents/Frameworks/Qt* lib/webots/qt include/qt bin/qt/lrelease bin/qt/lupdate bin/qt/moc

echo Done.
