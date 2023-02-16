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

#ifndef WB_SF_DOUBLE_HPP
#define WB_SF_DOUBLE_HPP

//
// Description: field value that contains a single double
//

#include "WbSingleValue.hpp"
#include "WbWriter.hpp"

#include "WbPrecision.hpp"

class WbSFDouble : public WbSingleValue {
  Q_OBJECT

public:
  WbSFDouble(WbTokenizer *tokenizer, const QString &worldPath) { readSFDouble(tokenizer, worldPath); }
  WbSFDouble(const WbSFDouble &other) : mValue(other.mValue) {}
  explicit WbSFDouble(double d) : mValue(d) {}
  virtual ~WbSFDouble() {}
  void read(WbTokenizer *tokenizer, const QString &worldPath) override { readSFDouble(tokenizer, worldPath); }
  void write(WbWriter &writer) const override {
    writer << toString(writer.isWebots() ? WbPrecision::DOUBLE_MAX : WbPrecision::FLOAT_MAX);
  }
  WbValue *clone() const override { return new WbSFDouble(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  WbVariant variantValue() const override { return WbVariant(mValue); }
  WbFieldType type() const override { return WB_SF_FLOAT; }
  double value() const { return mValue; }
  const double *valuePointer() const { return &mValue; }
  bool isZero() const { return mValue == 0.0; }
  void setValue(double d);
  void setValueNoSignal(double d) { mValue = d; }
  void setValueFromOde(double d) {
    mValue = d;
    emit changedByOde();
  }
  void add(double d);
  void mult(double factor);
  void makeAbsolute();                // absolute value
  bool clip(double min, double max);  // clip between min and max, return true if the value was changed
  WbSFDouble &operator=(const WbSFDouble &other);
  bool operator==(const WbSFDouble &other) const { return mValue == other.mValue; }

private:
  double mValue;
  void readSFDouble(WbTokenizer *tokenizer, const QString &worldPath);
};

#endif
