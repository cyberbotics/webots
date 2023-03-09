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

#ifndef WB_TELEMETRY_HPP
#define WB_TELEMETRY_HPP

//
// Description: this class reports telemetric information to Webots developers
//

#include <QtCore/QObject>

class WbTelemetry : public QObject {
  Q_OBJECT

public:
  static void send(const QString &operation, const QString &file = "");
  QString mFile;

private slots:
  void requestReplyFinished();

private:
  WbTelemetry() : mFile() {}
  virtual ~WbTelemetry() {}

  void sendRequest(const QString &operation);
};

#endif
