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

#ifndef WB_CLIPBOARD_HPP
#define WB_CLIPBOARD_HPP

//
// Description: Singleton class representing the application clipboard
//

#include "WbVariant.hpp"

#include <QtCore/QString>
#include <QtGui/QClipboard>

class WbField;
class QClipboard;

class WbClipboard : public WbVariant {
public:
  static WbClipboard *instance();
  static void deleteInstance();
  WbClipboard &operator=(const WbVariant &other);

  // redefine WbVariant setters
  void setBool(bool b) override;
  void setInt(int i) override;
  void setDouble(double d) override;
  void setString(const QString &s) override;
  void setVector2(const WbVector2 &v) override;
  void setVector3(const WbVector3 &v) override;
  void setColor(const WbRgb &c) override;
  void setRotation(const WbRotation &r) override;

  virtual bool isEmpty() const { return WbVariant::isEmpty() && !mNodeInfo; }
  void clear() override;

  // store copied node value as a string to be used when pasting
  // the isBoundingObjectNode property has to be set manually
  void setNode(WbNode *n, bool persistent = false) override;
  const QString &nodeExportString() const { return mNodeExportString; }
  QString computeNodeExportStringForInsertion(WbNode *parentNode, WbField *field, int fieldIndex) const;
  void replaceAllExternalDefNodesInString();

  struct WbClipboardNodeInfo {
    QString modelName;
    QString nodeModelName;
    QString slotType;
    bool hasADeviceDescendant;
    bool hasAConnectorDescendant;
  };
  const struct WbClipboardNodeInfo *nodeInfo() { return mNodeInfo; }

  QString stringValue();

  // synchronize Webots clipboard with system clipboard
  void update();

private:
  explicit WbClipboard();

  WbNode *toNode() const override { return NULL; }
  WbClipboardNodeInfo *mNodeInfo;
  QString mNodeExportString;
  struct LinkedDefNodeDefinitions {
    int position;  // USE position in string
    int type;
    QString defName;
    QString definition;
  };
  QList<struct LinkedDefNodeDefinitions *> mLinkedDefNodeDefinitions;
  int replaceExternalNodeDefinitionInString(QString &nodeString, int index) const;

  QClipboard *mSystemClipboard;
  QString mStringValue;
};

#endif
