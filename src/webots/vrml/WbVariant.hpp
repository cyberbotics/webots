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

#ifndef WB_VARIANT_HPP
#define WB_VARIANT_HPP

//
// Description: The WbVariant class acts like a union for Webots SF data types
//              Currently used to implement the Scene Tree's clipboard, but could be used in other situations
//

#include "WbPrecision.hpp"

#include <QtCore/QObject>

class WbRgb;
class WbNode;
class QString;
class WbVector2;
class WbVector3;
class WbRotation;

class WbVariant : public QObject {
  Q_OBJECT

public:
  WbVariant();
  explicit WbVariant(bool b);
  explicit WbVariant(int i);
  explicit WbVariant(double d);
  explicit WbVariant(const QString &s);
  explicit WbVariant(const WbVector2 &v);
  explicit WbVariant(const WbVector3 &v);
  explicit WbVariant(const WbRgb &c);
  explicit WbVariant(const WbRotation &r);
  explicit WbVariant(WbNode *n);
  WbVariant(const WbVariant &other);
  WbVariant &operator=(const WbVariant &other);
  bool operator==(const WbVariant &other) const;
  bool operator!=(const WbVariant &other) const;

  virtual ~WbVariant();

  int type() const { return mType; }
  bool isEmpty() const { return mType == -1; }
  virtual void clear();

  const QString toSimplifiedStringRepresentation(WbPrecision::Level level = WbPrecision::DOUBLE_MAX) const;

  // setters
  virtual void setBool(bool b);
  virtual void setInt(int i);
  virtual void setDouble(double d);
  virtual void setString(const QString &s);
  virtual void setVector2(const WbVector2 &v);
  virtual void setVector3(const WbVector3 &v);
  virtual void setColor(const WbRgb &c);
  virtual void setRotation(const WbRotation &r);
  virtual void setNode(WbNode *n, bool persistent = false);

  // getters
  bool toBool() const { return mBool; }
  int toInt() const { return mInt; }
  double toDouble() const { return mDouble; }
  const QString &toString() const { return *mString; }
  const WbVector2 &toVector2() const { return *mVector2; }
  const WbVector3 &toVector3() const { return *mVector3; }
  const WbRgb &toColor() const { return *mColor; }
  const WbRotation &toRotation() const { return *mRotation; }
  virtual WbNode *toNode() const { return mNode; }

protected:
  void setValue(const WbVariant &v);

protected slots:
  void clearNode();

private:
  int mType;
  bool mOwnsNode;
  union {
    bool mBool;
    int mInt;
    double mDouble;
    QString *mString;
    WbRgb *mColor;
    WbVector2 *mVector2;
    WbVector3 *mVector3;
    WbRotation *mRotation;
    WbNode *mNode;
  };
};

#endif
