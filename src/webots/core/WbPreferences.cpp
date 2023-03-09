// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbPreferences.hpp"

#include "WbLog.hpp"
#include "WbStandardPaths.hpp"
#include "WbSysInfo.hpp"

#ifdef _WIN32
#include "WbWindowsRegistry.hpp"
#endif

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QProcess>
#include <QtCore/QRegularExpressionMatch>
#include <QtCore/QStandardPaths>

static WbPreferences *gInstance = NULL;

WbPreferences *WbPreferences::createInstance(const QString &companyName, const QString &applicationName,
                                             const WbVersion &version) {
  if (gInstance)
    delete gInstance;

#ifdef __linux__
  if (WbSysInfo::isRootUser())
    QSettings::setPath(QSettings::NativeFormat, QSettings::UserScope, "/root/.config/");
#endif
  gInstance = new WbPreferences(companyName, applicationName, version);
  return gInstance;
}

WbPreferences *WbPreferences::instance() {
  return gInstance;
}

void WbPreferences::cleanup() {
  delete gInstance;
  gInstance = NULL;
}

WbPreferences::WbPreferences(const QString &companyName, const QString &applicationName, const WbVersion &version) :
  QSettings(QSettings::NativeFormat, QSettings::UserScope, companyName,
            QString("%1-%2").arg(applicationName).arg(version.toString(false))),
  mCompanyName(companyName),
  mApplicationName(applicationName),
  mVersion(version) {
  // use only one preferences file
  setFallbacksEnabled(false);
  // set defaults for preferences that are accessed from several locations
  setDefault("General/startupMode", "Real-time");
  setDefault("General/rendering", true);
  setDefault("General/language", "");
  setDefault("General/numberOfThreads", WbSysInfo::coreCount());
  setDefault("General/checkWebotsUpdateOnStartup", true);
  setDefault("General/disableSaveWarning", false);
  setDefault("General/thumbnail", true);
  setDefault("Sound/mute", true);
  setDefault("Sound/volume", 80);
  setDefault("OpenGL/disableShadows", false);
  setDefault("OpenGL/disableAntiAliasing", false);
  setDefault("OpenGL/GTAO", 2);
  setDefault("OpenGL/textureQuality", 4);
  setDefault("OpenGL/textureFiltering", 4);
  setDefault("VirtualRealityHeadset/enable", false);
  setDefault("VirtualRealityHeadset/trackPosition", true);
  setDefault("VirtualRealityHeadset/trackOrientation", true);
  setDefault("VirtualRealityHeadset/visibleEye", "left");
  setDefault("VirtualRealityHeadset/antiAliasing", false);
  setDefault("View3d/hideAllCameraOverlays", false);
  setDefault("View3d/hideAllRangeFinderOverlays", false);
  setDefault("View3d/hideAllDisplayOverlays", false);
  setDefault("Network/cacheSize", 1024);
  setDefault("Network/uploadUrl", "https://webots.cloud");
  setDefault("RobotWindow/newBrowserWindow", false);
  setDefault("RobotWindow/browser", "");

#ifdef _WIN32
  // "Monospace" isn't supported under Windows: the non-monospaced Arial font is loaded instead
  setDefault("Editor/font", "Consolas,10");
  setDefault("General/theme", "webots_classic.qss");
#elif defined(__APPLE__)
  setDefault("Editor/font", "Courier New,14");  // "Monospace" isn't supported under MacOS
  setDefault("General/theme", "webots_classic.qss");
#else
  setDefault("Editor/font", "Monospace, 9");
  setDefault("General/theme", "webots_night.qss");
#endif  // "Consolas" seems to be a standard Windows monospaced font, so we use it instead
  setDefault("Internal/firstLaunch", true);
  setDefault("Movie/resolution", 6);  // 480p: 854 x 480
  setDefault("Movie/quality", 90);
  setDefault("Movie/acceleration", 1.0);
  setDefault("Movie/caption", false);

  setDefault("Directories/projects", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/");
  setDefault("Directories/movies", QStandardPaths::writableLocation(QStandardPaths::MoviesLocation) + "/");
  setDefault("Directories/screenshots", QStandardPaths::writableLocation(QStandardPaths::PicturesLocation) + "/");
  setDefault("Directories/vrml", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/");
  setDefault("Directories/objects", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/");
  setDefault("Directories/www", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/");

  setDefaultPythonCommand();
}

WbPreferences::~WbPreferences() {
  setValue("Internal/firstLaunch", false);
}

void WbPreferences::setDefaultPythonCommand() {
#ifdef _WIN32
  const QString command = "python";
#else
  const QString command = "python3";
#endif
  QProcess process;
  process.start(command + WbStandardPaths::executableExtension(), QStringList() << "-c"
                                                                                << "print('PYTHON_COMMAND_FOUND');");
  process.waitForFinished();
  if (process.readAll().startsWith("PYTHON_COMMAND_FOUND")) {
    setDefault("General/pythonCommand", command);
    return;
  }
  setDefault("General/pythonCommand", "");
}

void WbPreferences::setDefault(const QString &key, const QVariant &value) {
  if (!contains(key))
    setValue(key, value);
}

void WbPreferences::setMoviePreferences(int resolutionIndex, int quality, double acceleration, bool caption) {
  setValue("Movie/resolution", resolutionIndex);
  setValue("Movie/quality", quality);
  setValue("Movie/acceleration", acceleration);
  setValue("Movie/caption", caption);
}

void WbPreferences::moviePreferences(int &resolutionIndex, int &quality, double &acceleration, bool &caption) const {
  resolutionIndex = value("Movie/resolution").toInt();
  quality = value("Movie/quality").toInt();
  acceleration = value("Movie/acceleration").toDouble();
  caption = value("Movie/caption").toBool();
}

QString WbPreferences::accessErrorString() const {
  if (status() == QSettings::AccessError) {
    if (QFile::exists(fileName()))
      return "<font color=\"red\">" + tr("Errors when accessing the preferences file.") + "<br>" +
             tr("If the error persists, please check your access rights on file:") + "<br>" + fileName() + "</font>";
  }

  return QString();
}

QString WbPreferences::findPreviousSettingsLocation() const {
  QStringList potentialLocations;

#ifdef _WIN32
  const QString registryRootLocation = QString("\\HKEY_CURRENT_USER\\SOFTWARE\\%1\\").arg(mCompanyName);
  potentialLocations = WbWindowsRegistry(registryRootLocation).subKeys();
  potentialLocations.replaceInStrings(QRegularExpression("^"), registryRootLocation);
#else

#ifdef __APPLE__
  QDir preferencesDirectory(QStandardPaths::writableLocation(QStandardPaths::ConfigLocation));
#else  // __linux__
  QDir preferencesDirectory(QStandardPaths::writableLocation(QStandardPaths::ConfigLocation) + "/" + mCompanyName);
#endif

  preferencesDirectory.setFilter(QDir::Files);
  QStringList filters;
#ifdef __APPLE__
  filters << QString("com.%1.%2*.plist").arg(mCompanyName.toLower()).arg(mApplicationName);
#else  // __linux__
  filters << QString("%1*.conf").arg(mApplicationName);
#endif
  preferencesDirectory.setNameFilters(filters);

  QFileInfoList preferencesFileInfos = preferencesDirectory.entryInfoList();
  for (int i = 0; i < preferencesFileInfos.size(); ++i) {
    QFileInfo preferencesFileInfo = preferencesFileInfos.at(i);
    potentialLocations << preferencesFileInfo.absoluteFilePath();
  }
#endif

  QString lastLocation;
  WbVersion lastLocationVersion;

  foreach (const QString &location, potentialLocations) {
    QFileInfo preferencesFileInfo(location);
    QRegularExpressionMatch match;
    WbVersion versionOfMatchedFile;
    WbVersion versionToTest(mVersion);
    if (preferencesFileInfo.fileName().contains(QRegularExpression("R\\d+\\w+"), &match))
      versionToTest.setRevision(0);  // the maintenance version should not be present when testing
    else if (preferencesFileInfo.fileName().contains(QRegularExpression("\\d+\\.\\d+\\.\\d+"), &match)) {
    } else if (preferencesFileInfo.fileName().contains(QRegularExpression("\\d+\\.\\d+"), &match))
      versionToTest.setRevision(0);  // the maintenance version should not be present when testing
    else
      // file name doesn't match any expected config file pattern
      continue;

    versionOfMatchedFile.fromString(match.captured());
    if (versionOfMatchedFile > lastLocationVersion && versionOfMatchedFile < versionToTest) {
      lastLocationVersion = versionOfMatchedFile;
      lastLocation = location;
    }
  }

  return lastLocation;
}

#ifdef __linux__
void WbPreferences::checkIsWritable() {
  if (!isWritable())
    WbLog::warning(tr("\nPreferences file cannot be overwritten.\n"
                      "Any change to the current settings won't be restored at next Webots start.\n\n"
                      "Please check the write permissions on file:\n\"%1\"")
                     .arg(fileName()),
                   true);
}
#endif

bool WbPreferences::booleanEnvironmentVariable(const QByteArray &variable) {
  const QByteArray content = qgetenv(variable).toLower();
  return !content.isEmpty() && content != "0" && content != "false";
}
