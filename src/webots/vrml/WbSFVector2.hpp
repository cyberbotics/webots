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

#ifndef WB_SF_VECTOR2_HPP
#define WB_SF_VECTOR2_HPP

//
// Description: field value that contains a single WbVector2
//

#include "WbSingleValue.hpp"
#include "WbVector2.hpp"
#include "WbWriter.hpp"

class WbSFVector2 : public WbSingleValue {
  Q_OBJECT

public:
  WbSFVector2(WbTokenizer *tokenizer, const QString &worldPath) { readSFVector2(tokenizer, worldPath); }
  WbSFVector2(const WbSFVector2 &other) : mValue(other.mValue) {}
  virtual ~WbSFVector2() {}
  void read(WbTokenizer *tokenizer, const QString &worldPath) override { readSFVector2(tokenizer, worldPath); }
  void write(WbWriter &writer) const override {
    writer << toString(writer.isWebots() ? WbPrecision::DOUBLE_MAX : WbPrecision::FLOAT_MAX);
  }
  WbValue *clone() const override { return new WbSFVector2(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  WbVariant variantValue() const override { return WbVariant(mValue); }
  WbFieldType type() const override { return WB_SF_VEC2F; }
  const WbVector2 &value() const { return mValue; }
  double x() const { return mValue.x(); }
  double y() const { return mValue.y(); }
  void setValue(const WbVector2 &v);
  void setValue(double x, double y);
  void setValue(const double xy[2]);
  void setValueFromWebots(const WbVector2 &v);
  void setX(double x) { setComponent(0, x); }
  void setY(double y) { setComponent(1, y); }
  double component(int index) const { return mValue[index]; }
  void setComponent(int index, double d);
  void mult(double factor);
  WbSFVector2 &operator=(const WbSFVector2 &other);
  bool operator==(const WbSFVector2 &other) const { return mValue == other.mValue; }

private:
  WbVector2 mValue;
  void readSFVector2(WbTokenizer *tokenizer, const QString &worldPath);
};

#endif
