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

#ifndef WB_QJS_FILE_HPP
#define WB_QJS_FILE_HPP

#include <QtCore/QObject>

class WbQjsFile : public QObject {
  Q_OBJECT

public:
  Q_INVOKABLE WbQjsFile(){};
  ~WbQjsFile(){};

  Q_INVOKABLE bool fileExists(const QString &filePath);
  Q_INVOKABLE QString readTextFile(const QString &filePath);
  Q_INVOKABLE bool writeTextFile(const QString &fileName, const QString &content);
};

#endif
