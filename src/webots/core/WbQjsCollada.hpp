// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WB_QJS_COLLDA_HPP
#define WB_QJS_COLLDA_HPP

#include <QtCore/QObject>

class WbQjsCollada : public QObject {
  Q_OBJECT

public:
  ~WbQjsCollada(){};
  Q_INVOKABLE static WbQjsCollada *instance();
  Q_INVOKABLE WbQjsCollada(){};

  Q_INVOKABLE QString getVrmlFromFile(const QString &filePath);

signals:
  void vrmlFromFileRequested(const QString &filePath);

public:
  void setVrmlResponse(const QString &vrml) { WbQjsCollada::cVrmlResponse = vrml; };

private:
  static QString cVrmlResponse;
  static WbQjsCollada *cInstance;
};

#endif
