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

#ifndef WB_SF_ROTATION_HPP
#define WB_SF_ROTATION_HPP

//
// Description: field value that contains a single WbRotation
//

#include "WbRotation.hpp"
#include "WbSingleValue.hpp"
#include "WbWriter.hpp"

class WbSFRotation : public WbSingleValue {
  Q_OBJECT

public:
  WbSFRotation() {}
  WbSFRotation(WbTokenizer *tokenizer, const QString &worldPath) { readSFRotation(tokenizer, worldPath); }
  WbSFRotation(const WbSFRotation &other) : mValue(other.mValue) {}
  explicit WbSFRotation(const WbRotation &r) : mValue(r) {}
  virtual ~WbSFRotation() {}
  void read(WbTokenizer *tokenizer, const QString &worldPath) override { readSFRotation(tokenizer, worldPath); }
  void write(WbWriter &writer) const override {
    writer << toString(writer.isWebots() ? WbPrecision::DOUBLE_MAX : WbPrecision::FLOAT_MAX);
  }
  WbValue *clone() const override { return new WbSFRotation(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  WbVariant variantValue() const override { return WbVariant(mValue); }
  WbFieldType type() const override { return WB_SF_ROTATION; }
  const WbRotation &value() const { return mValue; }
  double x() const { return mValue.x(); }
  double y() const { return mValue.y(); }
  double z() const { return mValue.z(); }
  double angle() const { return mValue.angle(); }
  void setValue(const WbRotation &v);
  void inline setValueFromOde(const WbRotation &v);
  void setValue(double x, double y, double z, double angle);
  void inline setValueFromOde(double x, double y, double z, double angle);
  void setValueByUser(const WbRotation &v, bool changedFromSupervisor);
  void setX(double x) { setComponent(0, x); }
  void setY(double y) { setComponent(1, y); }
  void setZ(double z) { setComponent(2, z); }
  void setAngle(double angle) { setComponent(3, angle); }
  void setComponent(int index, double d);
  double component(int index) const;
  WbSFRotation &operator=(const WbSFRotation &other);
  bool operator==(const WbSFRotation &other) const { return mValue == other.mValue; }

private:
  WbRotation mValue;
  void readSFRotation(WbTokenizer *tokenizer, const QString &worldPath);
};

void inline WbSFRotation::setValueFromOde(double x, double y, double z, double angle) {
  if (mValue == WbRotation(x, y, z, angle))
    return;

  mValue.setAxisAngle(x, y, z, angle);
  emit changedByOde();
}

void inline WbSFRotation::setValueFromOde(const WbRotation &v) {
  if (mValue == v)
    return;

  mValue = v;
  emit changedByOde();
}

#endif
