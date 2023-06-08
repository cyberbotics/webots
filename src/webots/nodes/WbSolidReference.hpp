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

//
//  WbSolidReference.hpp
//

// Node class representing a pointer to a Solid node
// It is used in the SFNode endPoint of a Joint to allow mechanical loop

#ifndef WB_SOLID_REFERENCE_HPP
#define WB_SOLID_REFERENCE_HPP

#include <QtCore/QPointer>
#include "WbBaseNode.hpp"
#include "WbSFString.hpp"

class WbSolid;

class WbSolidReference : public WbBaseNode {
  Q_OBJECT

public:
  explicit WbSolidReference(WbTokenizer *tokenizer = NULL);
  WbSolidReference(const WbSolidReference &other);
  explicit WbSolidReference(const WbNode &other);
  virtual ~WbSolidReference();

  int nodeType() const override { return WB_NODE_SOLID_REFERENCE; }
  void preFinalize() override;
  void postFinalize() override;

  QPointer<WbSolid> solid() const { return mSolid; }
  const QString &name() const { return mName->value(); }

  void updateName();
  bool pointsToStaticEnvironment() const { return mName->value() == STATIC_ENVIRONMENT; }
  static const QString STATIC_ENVIRONMENT;

  QList<const WbBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override;

  QString endPointName() const override;

signals:
  void changed();

private:
  WbSolidReference &operator=(const WbSolidReference &);  // non copyable
  WbSFString *mName;
  QPointer<WbSolid> mSolid;

private slots:
  void init();
};

#endif
