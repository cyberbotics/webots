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

#ifndef WB_FIELD_VALUE_RESTRICTION_HPP
#define WB_FIELD_VALUE_RESTRICTION_HPP

#include <QtCore/QString>
#include <WbNodeModel.hpp>
#include <WbProtoModel.hpp>
#include <WbVariant.hpp>

#include "../../../include/controller/c/webots/supervisor.h"

class WbFieldValueRestriction : public WbVariant {
  Q_OBJECT;

public:
  WbFieldValueRestriction() : WbVariant(), mAllowsSubtypes(false) {}
  explicit WbFieldValueRestriction(bool b) : WbVariant(b), mAllowsSubtypes(false) {}
  explicit WbFieldValueRestriction(int i) : WbVariant(i), mAllowsSubtypes(false) {}
  explicit WbFieldValueRestriction(double d) : WbVariant(d), mAllowsSubtypes(false) {}
  explicit WbFieldValueRestriction(const QString &s) : WbVariant(s), mAllowsSubtypes(false) {}
  explicit WbFieldValueRestriction(const WbVector2 &v) : WbVariant(v), mAllowsSubtypes(false) {}
  explicit WbFieldValueRestriction(const WbVector3 &v) : WbVariant(v), mAllowsSubtypes(false) {}
  explicit WbFieldValueRestriction(const WbRgb &c) : WbVariant(c), mAllowsSubtypes(false) {}
  explicit WbFieldValueRestriction(const WbRotation &r) : WbVariant(r), mAllowsSubtypes(false) {}
  explicit WbFieldValueRestriction(WbNode *n, bool allowsSubtypes) : WbVariant(n), mAllowsSubtypes(allowsSubtypes) {}
  explicit WbFieldValueRestriction(const WbVariant &variant, bool allowsSubtypes) :
    WbVariant(variant),
    mAllowsSubtypes(allowsSubtypes && variant.type() == WB_SF_NODE) {}
  WbFieldValueRestriction &operator=(const WbFieldValueRestriction &other);
  bool operator==(const WbFieldValueRestriction &other) const;
  bool operator!=(const WbFieldValueRestriction &other) const;

  virtual ~WbFieldValueRestriction() override {}

  const bool allowsSubtypes() const { return mAllowsSubtypes; }

  bool isVariantAccepted(const WbVariant &variant) const;
  bool isNodeAccepted(const QString &nodeModelName, const WbNodeModel *nodeModel, const QStringList &protoParentList) const;
  bool isNodeAccepted(const WbNode *node) const;
  bool isBaseNodeTypeAccepted(const WbNodeModel *actualType) const;
  // Note: This does not check if the proto's base type would be accepted by isBaseNodeTypeAccepted
  bool isProtoNodeTypeAccepted(const WbProtoModel *actualType) const;

private:
  // If the restriction is a node type, whether the node type can be a model which "extends" the requested type
  bool mAllowsSubtypes;
};

#endif
