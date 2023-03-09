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

#ifndef WB_PREFERENCES_HPP
#define WB_PREFERENCES_HPP

//
// Description: non-GUI part of Webots preferences
//   This singleton should be use by any class that needs to save/reload preferences
//

#include <QtCore/QSettings>

#include "WbVersion.hpp"

class WbPreferences : public QSettings {
  Q_OBJECT

public:
  static WbPreferences *createInstance(const QString &companyName, const QString &applicationName, const WbVersion &version);
  static WbPreferences *instance();
  static void cleanup();
  static bool booleanEnvironmentVariable(const QByteArray &variable);
  void setMoviePreferences(int resolutionIndex, int quality, double acceleration, bool caption);
  void moviePreferences(int &resolutionIndex, int &quality, double &acceleration, bool &caption) const;
  QString accessErrorString() const;
#ifdef __linux__
  // popup a warning message if the preferences file is not writable
  void checkIsWritable();
#endif

signals:
  void changedByUser();

private:
  WbPreferences(const QString &companyName, const QString &applicationName, const WbVersion &version);
  ~WbPreferences();

  QString findPreviousSettingsLocation() const;
  void setDefault(const QString &key, const QVariant &value);
  void setDefaultPythonCommand();

  QString mCompanyName;
  QString mApplicationName;
  WbVersion mVersion;
};

#endif
