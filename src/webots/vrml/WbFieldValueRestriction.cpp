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

#include "WbFieldValueRestriction.hpp"

#include "WbNode.hpp"

WbFieldValueRestriction &WbFieldValueRestriction::operator=(const WbFieldValueRestriction &other) {
  WbVariant::operator=(other);
  mAllowsSubtypes = other.allowsSubtypes();
  return *this;
}

bool WbFieldValueRestriction::operator==(const WbFieldValueRestriction &other) const {
  return WbVariant::operator==(other) && allowsSubtypes() == other.allowsSubtypes();
}

bool WbFieldValueRestriction::operator!=(const WbFieldValueRestriction &other) const {
  return !(*this == other);
}

bool WbFieldValueRestriction::isVariantAccepted(const WbVariant &variant) const {
  if (type() != variant.type())
    return false;
  if (type() != WB_SF_NODE)
    return variant == *this;
  return isNodeAccepted(variant.toNode());
}

bool WbFieldValueRestriction::isNodeAccepted(const QString &nodeModelName, const WbNodeModel *nodeModel,
                                             const QStringList &protoParentList) const {
  if (type() != WB_SF_NODE || !toNode() || !nodeModel)
    return false;
  return nodeModelName == toNode()->modelName() || isBaseNodeTypeAccepted(nodeModel) ||
         (allowsSubtypes() && protoParentList.contains(toNode()->modelName()));
}

bool WbFieldValueRestriction::isNodeAccepted(const WbNode *node) const {
  if (type() != WB_SF_NODE)
    return false;
  if (!toNode() || !node)
    return toNode() == node;
  return toNode()->isProtoInstance() ? isProtoNodeTypeAccepted(node->proto()) : isBaseNodeTypeAccepted(node->model());
}

bool WbFieldValueRestriction::isBaseNodeTypeAccepted(const WbNodeModel *actualType) const {
  if (type() != WB_SF_NODE || !toNode() || !actualType)
    return false;
  return toNode()->modelName() == actualType->name() || (allowsSubtypes() && isBaseNodeTypeAccepted(actualType->parentModel()));
}

bool WbFieldValueRestriction::isProtoNodeTypeAccepted(const WbProtoModel *actualType) const {
  if (type() != WB_SF_NODE || !toNode() || !actualType)
    return false;
  return toNode()->modelName() == actualType->name() ||
         (allowsSubtypes() && isProtoNodeTypeAccepted(actualType->ancestorProtoModel()));
}
