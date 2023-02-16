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

#include "WbVariant.hpp"

#include "WbNode.hpp"
#include "WbRgb.hpp"
#include "WbRotation.hpp"
#include "WbVector2.hpp"
#include "WbVector3.hpp"

#include <QtCore/QString>
#include "../../../include/controller/c/webots/supervisor.h"

WbVariant::WbVariant() : mType(-1), mOwnsNode(false) {
}

WbVariant::WbVariant(const WbVariant &other) : mType(-1), mOwnsNode(false) {
  setValue(other);
}

WbVariant &WbVariant::operator=(const WbVariant &other) {
  setValue(other);
  return *this;
}

bool WbVariant::operator==(const WbVariant &other) const {
  if (other.type() != mType)
    return false;
  if (mType == -1)
    return true;

  switch (mType) {
    case WB_SF_BOOL:
      return other.toBool() == mBool;
    case WB_SF_INT32:
      return other.toInt() == mInt;
    case WB_SF_FLOAT:
      return other.toDouble() == mDouble;
    case WB_SF_STRING:
      return other.toString() == *mString;
    case WB_SF_VEC2F:
      return other.toVector2() == *mVector2;
    case WB_SF_VEC3F:
      return other.toVector3() == *mVector3;
    case WB_SF_COLOR:
      return other.toColor() == *mColor;
    case WB_SF_ROTATION:
      return other.toRotation() == *mRotation;
    case WB_SF_NODE: {
      if (!mNode || !other.toNode())
        return mNode == other.toNode();
      return *other.toNode() == *mNode;
    }
    default:
      assert(false);
  }
  return false;
}

bool WbVariant::operator!=(const WbVariant &other) const {
  return !(*this == other);
}

WbVariant::WbVariant(bool b) : mType(-1), mOwnsNode(false) {
  WbVariant::setBool(b);
}

WbVariant::WbVariant(int i) : mType(-1), mOwnsNode(false) {
  WbVariant::setInt(i);
}

WbVariant::WbVariant(double d) : mType(-1), mOwnsNode(false) {
  WbVariant::setDouble(d);
}

WbVariant::WbVariant(const QString &s) : mType(-1), mOwnsNode(false) {
  WbVariant::setString(s);
}

WbVariant::WbVariant(const WbVector2 &v) : mType(-1), mOwnsNode(false) {
  WbVariant::setVector2(v);
}

WbVariant::WbVariant(const WbVector3 &v) : mType(-1), mOwnsNode(false) {
  WbVariant::setVector3(v);
}

WbVariant::WbVariant(const WbRgb &c) : mType(-1), mOwnsNode(false) {
  WbVariant::setColor(c);
}

WbVariant::WbVariant(const WbRotation &r) : mType(-1), mOwnsNode(false) {
  WbVariant::setRotation(r);
}

WbVariant::WbVariant(WbNode *n) : mType(-1), mOwnsNode(false) {
  WbVariant::setNode(n);
}

WbVariant::~WbVariant() {
  WbVariant::clear();
}

void WbVariant::clear() {
  switch (mType) {
    case WB_SF_STRING:
      delete mString;
      break;
    case WB_SF_VEC2F:
      delete mVector2;
      break;
    case WB_SF_VEC3F:
      delete mVector3;
      break;
    case WB_SF_COLOR:
      delete mColor;
      break;
    case WB_SF_ROTATION:
      delete mRotation;
      break;
    case WB_SF_NODE:
      if (mNode && mOwnsNode) {
        delete mNode;
        mNode = NULL;
        mOwnsNode = false;
      }
      break;
  }

  mType = -1;
}

const QString WbVariant::toSimplifiedStringRepresentation(WbPrecision::Level level) const {
  switch (mType) {
    case WB_SF_BOOL:
      return mBool ? "TRUE" : "FALSE";
    case WB_SF_INT32:
      return QString::number(mInt);
    case WB_SF_FLOAT:
      return WbPrecision::doubleToString(mDouble, level);
    case WB_SF_STRING:
      return QString("\"%1\"").arg(*mString);
    case WB_SF_VEC2F:
      return mVector2->toString(level);
    case WB_SF_VEC3F:
      return mVector3->toString(level);
    case WB_SF_COLOR:
      return mColor->toString(level);
    case WB_SF_ROTATION:
      return mRotation->toString(level);
    case WB_SF_NODE: {
      if (mNode)
        return mNode->fullName();
      else
        return "NULL";
    }
    default:
      assert(false);
  }
  return "";
}

void WbVariant::setBool(bool b) {
  clear();
  mBool = b;
  mType = WB_SF_BOOL;
}

void WbVariant::setInt(int i) {
  clear();
  mInt = i;
  mType = WB_SF_INT32;
}

void WbVariant::setDouble(double d) {
  clear();
  mDouble = d;
  mType = WB_SF_FLOAT;
}

void WbVariant::setVector2(const WbVector2 &v) {
  clear();
  mVector2 = new WbVector2(v);
  mType = WB_SF_VEC2F;
}

void WbVariant::setVector3(const WbVector3 &v) {
  clear();
  mVector3 = new WbVector3(v);
  mType = WB_SF_VEC3F;
}

void WbVariant::setString(const QString &s) {
  clear();
  mString = new QString(s);
  mType = WB_SF_STRING;
}

void WbVariant::setColor(const WbRgb &c) {
  mColor = new WbRgb(c);
  mType = WB_SF_COLOR;
}

void WbVariant::setRotation(const WbRotation &r) {
  clear();
  mRotation = new WbRotation(r);
  mType = WB_SF_ROTATION;
}

void WbVariant::setNode(WbNode *n, bool persistent) {
  clear();
  if (persistent) {
    // If persistent is true, the variant owns its own clone of the node.
    // This is usefull in case of enumeration for the field model.
    mNode = n ? n->cloneAndReferenceProtoInstance() : NULL;
    if (mNode)
      mOwnsNode = true;
  } else {
    mNode = n;
    if (n)
      connect(n, &QObject::destroyed, this, &WbVariant::clearNode);
  }
  mType = WB_SF_NODE;
}

void WbVariant::setValue(const WbVariant &v) {
  switch (v.type()) {
    case WB_SF_STRING:
      setString(v.toString());
      break;
    case WB_SF_VEC2F:
      setVector2(v.toVector2());
      break;
    case WB_SF_VEC3F:
      setVector3(v.toVector3());
      break;
    case WB_SF_COLOR:
      setColor(v.toColor());
      break;
    case WB_SF_ROTATION:
      setRotation(v.toRotation());
      break;
    case WB_SF_BOOL:
      setBool(v.toBool());
      break;
    case WB_SF_INT32:
      setInt(v.toInt());
      break;
    case WB_SF_FLOAT:
      setDouble(v.toDouble());
      break;
    case WB_SF_NODE:
      setNode(v.toNode());
      break;
  }
}

void WbVariant::clearNode() {
  mNode = NULL;
}
