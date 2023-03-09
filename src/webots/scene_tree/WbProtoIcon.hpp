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

#ifndef WB_PROTO_ICON_HPP
#define WB_PROTO_ICON_HPP

//
// Description: object representing the PROTO icon and handling its download
//

#include <QtCore/QObject>

class WbDownloader;

class QDir;

class WbProtoIcon : public QObject {
  Q_OBJECT

public:
  explicit WbProtoIcon(const QString &modelName, const QString &protoPath, QObject *parent = NULL);

  const QString &path() const { return mPath; }
  bool isReady() const { return mReady; }

  void duplicate(QDir destinationDir);

signals:
  void iconReady(const QString &path);

private:
  QString mPath;
  const QString mModelName;
  WbDownloader *mDownloader;
  bool mReady;

private slots:
  void updateIcon();
};

#endif
