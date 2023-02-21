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

#ifndef WB_NODE_MODEL_HPP
#define WB_NODE_MODEL_HPP

//
// Description: a class for managing built-in node definitions stored in .wrl files
//

#include <QtCore/QList>
#include <QtCore/QMap>
#include <QtCore/QString>
#include <QtCore/QStringList>

class WbFieldModel;
class WbTokenizer;

class WbNodeModel {
public:
  // static functions
  static WbNodeModel *findModel(const QString &modelName);
  static bool fuzzyParseNode(const QString &fileName, QString &nodeInfo);

  // alphabetical list of all nodes, i.e. ("Accelerometer", "Appearance", "Background", "Box" ...)
  static QStringList baseModelNames();
  static bool isBaseModelName(const QString &modelName);

  // backward compatibility
  static QString compatibleNodeName(const QString &modelName);

  // node name, e.g. "Transform", "Solid" ...
  const QString &name() const { return mName; }

  // the info comments (#) found at the beginning of the .wrl or .proto file
  const QString &info() const { return mInfo; }

  // field models
  WbFieldModel *findFieldModel(const QString &fieldName) const;
  const QList<WbFieldModel *> &fieldModels() const { return mFieldModels; }
  QStringList fieldNames();

  QStringList documentationBookAndPage() const { return QStringList() << "reference" << mName.toLower(); }

private:
  explicit WbNodeModel(WbTokenizer *tokenizer);
  ~WbNodeModel();

  QString mInfo;
  QString mName;
  QList<WbFieldModel *> mFieldModels;

  static WbNodeModel *readModel(const QString &fileName);
  static void readAllModels();
  static void cleanup();
  static QMap<QString, WbNodeModel *> cModels;
};

#endif
