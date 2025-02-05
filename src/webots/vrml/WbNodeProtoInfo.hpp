// Copyright 1996-2024 Cyberbotics Ltd.
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

#ifndef WB_NODE_PROTO_ANCESTRY_HPP
#define WB_NODE_PROTO_ANCESTRY_HPP

//
// Description: class containing proto-specific information about a VRML node
//

#include <QtCore/QList>
#include <QtCore/QString>

class WbField;

struct WbFieldReference {
  QString name;
  WbField *actualField;
};

class WbNodeProtoInfo {
public:
  WbNodeProtoInfo(const QString &modelName, const QList<WbField *> &parameters);
  explicit WbNodeProtoInfo(const WbNodeProtoInfo &other);

  const QString &modelName() const { return mModelName; }
  const QList<WbFieldReference> &parameters() const { return mParameters; }

  const WbFieldReference &findFieldByIndex(int index) const;
  int findFieldIndex(const QString &name) const;

  void redirectFields(const WbField *oldField, WbField *newField);

private:
  WbNodeProtoInfo &operator=(const WbNodeProtoInfo &);  // non copyable

  QString mModelName;
  QList<WbFieldReference> mParameters;
};

#endif
