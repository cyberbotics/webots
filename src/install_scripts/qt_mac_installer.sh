QT_VERSION=5.11.2
WEBOTS_PATH=~/develop/webots

# prepare Webots
cd $WEBOTS_PATH
rm -fr Contents/Frameworks/Qt* bin/qt/lupdate bin/qt/lrelease bin/qt/moc include/qt lib/qt
mkdir lib/qt
mkdir include/qt

# populate webots
cd $HOME/Qt$QT_VERSION/$QT_VERSION/clang_64
cp bin/lrelease $WEBOTS_PATH/bin/qt
cp bin/lupdate $WEBOTS_PATH/bin/qt
cp bin/moc $WEBOTS_PATH/bin/qt
cp -r include/QtPlatformHeaders $WEBOTS_PATH/include/qt/

declare -a qtFrameworks=( \
  "QtConcurrent" \
  "QtCore" \
  "QtDBus" \
  "QtGui" \
  "QtMultimedia" \
  "QtMultimediaWidgets" \
  "QtNetwork" \
  "QtOpenGL" \
  "QtPositioning" \
  "QtPrintSupport" \
  "QtQml" \
  "QtQuick" \
  "QtQuickWidgets" \
  "QtSensors" \
  "QtSql" \
  "QtWebChannel" \
  "QtWebEngine" \
  "QtWebEngineCore" \
  "QtWebEngineWidgets" \
  "QtWebSockets" \
  "QtWidgets" \
  "QtXml" \
)

for f in "${qtFrameworks[@]}"
do
  cp -R lib/$f.framework $WEBOTS_PATH/Contents/Frameworks
done

mkdir $WEBOTS_PATH/lib/qt/plugins
mkdir $WEBOTS_PATH/lib/qt/plugins/imageformats
mkdir $WEBOTS_PATH/lib/qt/plugins/platforms
mkdir $WEBOTS_PATH/lib/qt/plugins/printsupport
mkdir $WEBOTS_PATH/lib/qt/plugins/styles
mkdir $WEBOTS_PATH/lib/qt/libexec
cp plugins/imageformats/libqjpeg.dylib $WEBOTS_PATH/lib/qt/plugins/imageformats/
cp plugins/platforms/libqcocoa.dylib $WEBOTS_PATH/lib/qt/plugins/platforms/
cp plugins/printsupport/libcocoaprintersupport.dylib $WEBOTS_PATH/lib/qt/plugins/printsupport/
cp plugins/styles/libqmacstyle.dylib $WEBOTS_PATH/lib/qt/plugins/styles/
cp ../../Examples/Qt-$QT_VERSION/webchannel/shared/qwebchannel.js $WEBOTS_PATH/resources/web/local/qwebchannel.js
echo $'[Paths]\nPrefix = ..\n' > $WEBOTS_PATH/lib/qt/libexec/qt.conf

# remove the debug frameworks
cd  $WEBOTS_PATH/Contents/Frameworks/
find . -name *_debug | xargs rm

# Render the frameworks relative to the executable:
cd  $WEBOTS_PATH/Contents/Frameworks/

for fA in "${qtFrameworks[@]}"
do
   install_name_tool -id @rpath/Contents/Frameworks/$fA.framework/Versions/5/$fA $fA.framework/Versions/5/$fA
   install_name_tool -delete_rpath @executable_path/Frameworks $fA.framework/Versions/5/$fA
   install_name_tool -delete_rpath @loader_path/Frameworks $fA.framework/Versions/5/$fA
   install_name_tool -add_rpath @loader_path/../../.. $fA.framework/Versions/5/$fA
   for fB in "${qtFrameworks[@]}"
   do
     install_name_tool -change @rpath/$fB.framework/Versions/5/$fB @rpath/Contents/Frameworks/$fB.framework/Versions/5/$fB $fA.framework/Versions/5/$fA
   done
done

# Render the plugins relative to the executable:
cd $WEBOTS_PATH/lib/qt/plugins

declare -a libs=("imageformats/libqjpeg.dylib" "platforms/libqcocoa.dylib" "printsupport/libcocoaprintersupport.dylib" "styles/libqmacstyle.dylib")

for lib in "${libs[@]}"
do
  install_name_tool -id @rpath/lib/qt/plugins/$lib $lib
  install_name_tool -delete_rpath @executable_path/Frameworks $lib
  install_name_tool -delete_rpath @loader_path/../../lib $lib
  install_name_tool -add_rpath @loader_path/../../../.. $lib
  for f in "${qtFrameworks[@]}"
  do
    install_name_tool -change @rpath/$f.framework/Versions/5/$f @rpath/Contents/Frameworks/$f.framework/Versions/5/$f $lib
  done
done

# Change the RPATH of the executables
cd $WEBOTS_PATH/bin/qt
install_name_tool -rpath @loader_path/../lib @loader_path/../.. lrelease
install_name_tool -change @rpath/QtCore.framework/Versions/5/QtCore @rpath/Contents/Frameworks/QtCore.framework/Versions/5/QtCore lrelease
install_name_tool -change @rpath/QtXml.framework/Versions/5/QtXml @rpath/Contents/Frameworks/QtXml.framework/Versions/5/QtXml lrelease
install_name_tool -rpath @loader_path/../lib @loader_path/../.. lupdate
install_name_tool -change @rpath/QtCore.framework/Versions/5/QtCore @rpath/Contents/Frameworks/QtCore.framework/Versions/5/QtCore lupdate
install_name_tool -change @rpath/QtXml.framework/Versions/5/QtXml @rpath/Contents/Frameworks/QtXml.framework/Versions/5/QtXml lupdate
install_name_tool -add_rpath @loader_path/../.. moc
cd $WEBOTS_PATH/Contents/Frameworks/QtWebEngineCore.framework/Helpers/QtWebEngineProcess.app/Contents/MacOS
install_name_tool -change @rpath/QtWebEngineCore.framework/Versions/5/QtWebEngineCore @rpath/Contents/Frameworks/QtWebEngineCore.framework/Versions/5/QtWebEngineCore QtWebEngineProcess
install_name_tool -change @rpath/QtQuick.framework/Versions/5/QtQuick @rpath/Contents/Frameworks/QtQuick.framework/Versions/5/QtQuick QtWebEngineProcess
install_name_tool -change @rpath/QtQml.framework/Versions/5/QtQml @rpath/Contents/Frameworks/QtQml.framework/Versions/5/QtQml QtWebEngineProcess
install_name_tool -change @rpath/QtNetwork.framework/Versions/5/QtNetwork @rpath/Contents/Frameworks/QtNetwork.framework/Versions/5/QtNetwork QtWebEngineProcess
install_name_tool -change @rpath/QtCore.framework/Versions/5/QtCore @rpath/Contents/Frameworks/QtCore.framework/Versions/5/QtCore QtWebEngineProcess
install_name_tool -change @rpath/QtGui.framework/Versions/5/QtGui @rpath/Contents/Frameworks/QtGui.framework/Versions/5/QtGui QtWebEngineProcess
install_name_tool -change @rpath/QtWebChannel.framework/Versions/5/QtWebChannel @rpath/Contents/Frameworks/QtWebChannel.framework/Versions/5/QtWebChannel QtWebEngineProcess
install_name_tool -change @rpath/QtPositioning.framework/Versions/5/QtPositioning @rpath/Contents/Frameworks/QtPositioning.framework/Versions/5/QtPositioning QtWebEngineProcess
install_name_tool -add_rpath @loader_path/../../../../../../../../.. QtWebEngineProcess

cd $WEBOTS_PATH

ARCHIVE=qt-$QT_VERSION-release.tar.bz2
echo Compressing $ARCHIVE \(please wait\)
tar cjf $ARCHIVE Contents/Frameworks/Qt* lib/qt include/qt bin/qt/lrelease bin/qt/lupdate bin/qt/moc resources/web/local/qwebchannel.js

echo Done.
