#!/bin/bash

if [[ -z "${WEBOTS_HOME}" ]]; then
  echo "WEBOTS_HOME is not defined."
  exit 1
fi

QT_VERSION=6.2.3

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
mkdir $WEBOTS_HOME/lib/webots/qt/plugins/printsupport
mkdir $WEBOTS_HOME/lib/webots/qt/plugins/styles
mkdir $WEBOTS_HOME/lib/webots/qt/plugins/tls
mkdir $WEBOTS_HOME/lib/webots/qt/libexec
cp plugins/imageformats/libqjpeg.dylib $WEBOTS_HOME/lib/webots/qt/plugins/imageformats/
cp plugins/platforms/libqcocoa.dylib $WEBOTS_HOME/lib/webots/qt/plugins/platforms/
cp plugins/printsupport/libcocoaprintersupport.dylib $WEBOTS_HOME/lib/webots/qt/plugins/printsupport/
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
   install_name_tool -id @rpath/Contents/Frameworks/$fA.framework/Versions/Current/$fA $fA.framework/Versions/Current/$fA
   install_name_tool -delete_rpath @executable_path/../Frameworks $fA.framework/Versions/Current/$fA
   install_name_tool -delete_rpath @loader_path/Frameworks $fA.framework/Versions/Current/$fA
   for fB in "${qtFrameworks[@]}"
   do
     install_name_tool -change @rpath/$fB.framework/Versions/Current/$fB @rpath/Contents/Frameworks/$fB.framework/Versions/Current/$fB $fA.framework/Versions/Current/$fA
   done
done

# Render the plugins relative to the executable:
cd $WEBOTS_HOME/lib/webots/qt/plugins

declare -a libs=("imageformats/libqjpeg.dylib" "platforms/libqcocoa.dylib" "printsupport/libcocoaprintersupport.dylib" "styles/libqmacstyle.dylib")

for lib in "${libs[@]}"
do
  install_name_tool -id @rpath/lib/webots/qt/plugins/$lib $lib
  install_name_tool -delete_rpath @executable_path/../Frameworks $lib
  install_name_tool -delete_rpath @loader_path/../../lib $lib
  install_name_tool -add_rpath @loader_path/../../../.. $lib
  for f in "${qtFrameworks[@]}"
  do
    install_name_tool -change @rpath/$f.framework/Versions/Current/$f @rpath/Contents/Frameworks/$f.framework/Versions/Current/$f $lib
  done
done

# Change the RPATH of the executables
cd $WEBOTS_HOME/bin/qt
install_name_tool -rpath @loader_path/../lib @loader_path/../.. lrelease
install_name_tool -change @rpath/QtCore.framework/Versions/Current/QtCore @rpath/Contents/Frameworks/QtCore.framework/Versions/Current/QtCore lrelease
install_name_tool -change @rpath/QtXml.framework/Versions/Current/QtXml @rpath/Contents/Frameworks/QtXml.framework/Versions/Current/QtXml lrelease
install_name_tool -rpath @loader_path/../lib @loader_path/../.. lupdate
install_name_tool -change @rpath/QtCore.framework/Versions/Current/QtCore @rpath/Contents/Frameworks/QtCore.framework/Versions/Current/QtCore lupdate
install_name_tool -change @rpath/QtXml.framework/Versions/Current/QtXml @rpath/Contents/Frameworks/QtXml.framework/Versions/Current/QtXml lupdate
install_name_tool -add_rpath @loader_path/../.. moc

cd $WEBOTS_HOME

ARCHIVE=qt-$QT_VERSION-release.tar.bz2
echo Compressing $ARCHIVE \(please wait\)
tar cjf $ARCHIVE Contents/Frameworks/Qt* lib/webots/qt include/qt bin/qt/lrelease bin/qt/lupdate bin/qt/moc

echo Done.
