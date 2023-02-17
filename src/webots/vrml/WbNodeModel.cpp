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

#include "WbNodeModel.hpp"

#include "WbFieldModel.hpp"
#include "WbLog.hpp"
#include "WbParser.hpp"
#include "WbStandardPaths.hpp"
#include "WbTokenizer.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QMap>
#include <QtCore/QRegularExpression>
#include <QtCore/QTextStream>

QMap<QString, WbNodeModel *> WbNodeModel::cModels;

void WbNodeModel::cleanup() {
  QMapIterator<QString, WbNodeModel *> it(cModels);
  while (it.hasNext())
    delete it.next().value();
}

WbNodeModel::WbNodeModel(WbTokenizer *tokenizer) : mInfo(tokenizer->info()), mName(tokenizer->nextWord()) {
  tokenizer->skipToken("{");

  while (tokenizer->peekWord() != "}") {
    WbFieldModel *fieldModel = new WbFieldModel(tokenizer, "");
    fieldModel->ref();
    mFieldModels.append(fieldModel);
  }

  tokenizer->skipToken("}");
}

WbNodeModel::~WbNodeModel() {
  foreach (WbFieldModel *fieldModel, mFieldModels)
    fieldModel->unref();
  mFieldModels.clear();
}

WbNodeModel *WbNodeModel::readModel(const QString &fileName) {
  WbTokenizer tokenizer;
  int errors = tokenizer.tokenize(fileName);
  if (errors > 0)
    return NULL;

  // parsing the .wrl files is not absolutely necessary:
  // we can assume that their syntax is correct
  WbParser parser(&tokenizer);
  bool ok = parser.parseNodeModel();
  if (!ok)
    return NULL;

  tokenizer.rewind();
  return new WbNodeModel(&tokenizer);
}

void WbNodeModel::readAllModels() {
  QString path = WbStandardPaths::resourcesPath() + "nodes/";
  QStringList list = QDir(path, "*.wrl").entryList();
  foreach (QString modelName, list) {
    WbNodeModel *model = readModel(path + modelName);
    if (model)
      cModels.insert(model->name(), model);
  }

  qAddPostRoutine(WbNodeModel::cleanup);
}

QStringList WbNodeModel::baseModelNames() {
  if (cModels.isEmpty())
    readAllModels();

  return cModels.keys();
}

bool WbNodeModel::isBaseModelName(const QString &modelName) {
  if (cModels.isEmpty())
    readAllModels();

  return cModels.contains(modelName);
}

QString WbNodeModel::compatibleNodeName(const QString &modelName) {
  if (modelName == "CameraFocus")
    return "Focus";
  if (modelName == "CameraLensDistortion")
    return "Lens";
  if (modelName == "CameraZoom")
    return "Zoom";
  return modelName;
}

WbNodeModel *WbNodeModel::findModel(const QString &modelName) {
  if (cModels.isEmpty())
    readAllModels();

  return cModels.value(modelName, NULL);
}

WbFieldModel *WbNodeModel::findFieldModel(const QString &fieldName) const {
  foreach (WbFieldModel *fieldModel, mFieldModels)
    if (fieldModel->name() == fieldName)
      return fieldModel;

  return NULL;  // not found
}

bool WbNodeModel::fuzzyParseNode(const QString &fileName, QString &nodeInfo) {
  QFile input(WbStandardPaths::resourcesPath() + "nodes/" + fileName + ".wrl");
  if (!input.open(QIODevice::ReadOnly)) {
    WbLog::warning(fileName + ": could not open file", false, WbLog::PARSING);
    return false;
  }

  nodeInfo.clear();
  QTextStream stream(&input);
  QString line;
  while (!stream.atEnd()) {
    line = stream.readLine();
    if (line.startsWith("#")) {
      if (line.contains(QRegularExpression("#\\s*VRML(_...|) V(\\d+).(\\d+)")))
        continue;
      line = line.mid(1).trimmed();  // clean line
      if (!line.isEmpty())
        nodeInfo.append(line + "\n");
    } else {
      nodeInfo.chop(1);  // remove last newline
      break;
    }
  }

  input.close();
  return true;
}

QStringList WbNodeModel::fieldNames() {
  QStringList names;
  foreach (WbFieldModel *fieldModel, mFieldModels)
    names.append(fieldModel->name());
  return names;
}
