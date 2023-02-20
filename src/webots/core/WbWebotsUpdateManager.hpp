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

#ifndef WB_WEBOTS_UPDATE_MANAGER_HPP
#define WB_WEBOTS_UPDATE_MANAGER_HPP

//
// Description: class allow to get the webots target version
//

#include <QtCore/QObject>

#include "WbVersion.hpp"

class WbWebotsUpdateManager : public QObject {
  Q_OBJECT

public:
  static WbWebotsUpdateManager *instance();

  bool isTargetVersionAvailable() { return mTargetVersionAvailable; }
  const WbVersion &targetVersion() { return mVersion; }

  const QString &error() { return mError; }

signals:
  void targetVersionAvailable();

private slots:
  void downloadReplyFinished();

private:
  WbWebotsUpdateManager();
  virtual ~WbWebotsUpdateManager();

  static void cleanup();

  static WbWebotsUpdateManager *cInstance;

  void sendRequest();

  WbVersion mVersion;
  bool mTargetVersionAvailable;
  QString mError;
};

#endif
