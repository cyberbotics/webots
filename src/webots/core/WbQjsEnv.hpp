// Copyright 1996-2025 Cyberbotics Ltd.
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

#ifndef WB_QJS_ENV_HPP
#define WB_QJS_ENV_HPP

#include <QtCore/QObject>

class WbQjsEnv : public QObject {
  Q_OBJECT

public:
  Q_INVOKABLE WbQjsEnv(){};
  ~WbQjsEnv(){};

  Q_INVOKABLE QString getFromEnv(const QString &name) const;
};

#endif
