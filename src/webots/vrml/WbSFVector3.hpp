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

#ifndef WB_SF_VECTOR3_HPP
#define WB_SF_VECTOR3_HPP

//
// Description: field value that contains a single WbVector3
//

#include "WbSingleValue.hpp"
#include "WbVector3.hpp"
#include "WbWriter.hpp"

class WbSFVector3 : public WbSingleValue {
  Q_OBJECT

public:
  WbSFVector3() {}
  WbSFVector3(WbTokenizer *tokenizer, const QString &worldPath) { readSFVector3(tokenizer, worldPath); }
  WbSFVector3(const WbSFVector3 &other) : mValue(other.mValue) {}
  explicit WbSFVector3(const WbVector3 &v) : mValue(v) {}
  virtual ~WbSFVector3() {}
  void read(WbTokenizer *tokenizer, const QString &worldPath) override { readSFVector3(tokenizer, worldPath); };
  void write(WbWriter &writer) const override {
    writer << toString(writer.isWebots() ? WbPrecision::DOUBLE_MAX : WbPrecision::FLOAT_MAX);
  }
  WbValue *clone() const override { return new WbSFVector3(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  WbVariant variantValue() const override { return WbVariant(mValue); }
  WbFieldType type() const override { return WB_SF_VEC3F; }
  const WbVector3 &value() const { return mValue; }
  double x() const { return mValue.x(); }
  double y() const { return mValue.y(); }
  double z() const { return mValue.z(); }
  bool isNull() const { return mValue.isNull(); }
  void setValue(const WbVector3 &v);
  void setValue(double x, double y, double z);
  void setValueNoSignal(double x, double y, double z) { mValue.setXyz(x, y, z); }
  void setValueFromOde(double x, double y, double z) {
    mValue.setXyz(x, y, z);
    emit changedByOde();
  }
  void setValueFromOde(const WbVector3 &v) {
    mValue.setXyz(v.x(), v.y(), v.z());
    emit changedByOde();
  }
  void setValueByUser(const WbVector3 &v, bool changedFromSupervisor);
  void setValue(const double xyz[3]);
  void setX(double x) { setComponent(0, x); }
  void setY(double y) { setComponent(1, y); }
  void setYnoSignal(double y) { mValue.setY(y); }
  void setZ(double z) { setComponent(2, z); }
  double component(int index) const { return mValue[index]; }
  void setComponent(int index, double d);
  void mult(double factor);
  WbSFVector3 &operator=(const WbSFVector3 &other);
  bool operator==(const WbSFVector3 &other) const { return mValue == other.mValue; }

private:
  WbVector3 mValue;
  void readSFVector3(WbTokenizer *tokenizer, const QString &worldPath);
};

#endif
