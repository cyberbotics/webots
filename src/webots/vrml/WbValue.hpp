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

#ifndef WB_VALUE_HPP
#define WB_VALUE_HPP

//
// Description:
//   WbValue is the abstract base class for any type of value that can be stored in a WbField
//
// Inherited by:
//   WbSingleValue, WbMultipleValue
//

#include <QtCore/QObject>
#include "../../../include/controller/c/webots/supervisor.h"

#include "WbPrecision.hpp"

class WbTokenizer;
class WbWriter;

class WbValue : public QObject {
  Q_OBJECT

public:
  virtual ~WbValue();

  // virtual copy constructor
  virtual WbValue *clone() const = 0;

  // read the value
  virtual void read(WbTokenizer *tokenizer, const QString &worldPath) = 0;

  // write the value
  virtual void write(WbWriter &writer) const = 0;

  // virtual comparison and assignment
  virtual bool equals(const WbValue *other) const = 0;
  virtual void copyFrom(const WbValue *other) = 0;
  void emitChangedByOde() { emit changedByOde(); }

  // string for the GUI
  // level is not meaningful in all the subclasses.
  virtual QString toString(WbPrecision::Level level = WbPrecision::DOUBLE_MAX) const = 0;

  // field type as used supervisor functions
  virtual WbFieldType type() const = 0;
  WbFieldType singleType() const;
  bool isSingle() const { return WbValue::isSingle(type()); }
  bool isMultiple() const { return WbValue::isMultiple(type()); }

  // e.g. "SFNode", "SFVec3f", "SFInt32", etc.
  QString vrmlTypeName() const;

  // e.g. "Node", "Vector3", "Int"
  QString shortTypeName() const;

  // static operation for the field type
  static bool isSingle(WbFieldType type);
  static bool isMultiple(WbFieldType type);
  static WbFieldType toSingle(WbFieldType type);
  static WbFieldType vrmlNameToType(const QString &vrmlName);
  static QString typeToVrmlName(WbFieldType type);
  static QString typeToShortName(WbFieldType type);

  virtual void defHasChanged() {}

signals:
  // emitted after the content of the value was changed
  void changed();
  void changedByOde();
  void changedByUser(bool changedFromSupervisor);
  void changedByWebots();

protected:
  // abstract class cannot be instantiated
  WbValue();

private:
};

#endif
