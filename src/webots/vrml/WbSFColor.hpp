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

#ifndef WB_SF_COLOR_HPP
#define WB_SF_COLOR_HPP

//
// Description: field value that contains a single WbRgb
//

#include "WbRgb.hpp"
#include "WbSingleValue.hpp"
#include "WbWriter.hpp"

class WbSFColor : public WbSingleValue {
  Q_OBJECT

public:
  explicit WbSFColor(double r, double g, double b) : mValue(WbRgb(r, g, b)) {}
  WbSFColor(WbTokenizer *tokenizer, const QString &worldPath) { readSFColor(tokenizer, worldPath); }
  WbSFColor(const WbSFColor &other) : mValue(other.mValue) {}
  virtual ~WbSFColor() {}
  void read(WbTokenizer *tokenizer, const QString &worldPath) override { readSFColor(tokenizer, worldPath); }
  void write(WbWriter &writer) const override {
    writer << toString(writer.isWebots() ? WbPrecision::DOUBLE_MAX : WbPrecision::FLOAT_MAX);
  }
  WbValue *clone() const override { return new WbSFColor(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  WbVariant variantValue() const override { return WbVariant(mValue); }
  WbFieldType type() const override { return WB_SF_COLOR; }
  const WbRgb &value() const { return mValue; }
  double red() const { return mValue.red(); }
  double green() const { return mValue.green(); }
  double blue() const { return mValue.blue(); }
  void setValue(const WbRgb &c);
  void setValue(double r, double g, double b);     // values between 0.0 and 1.0
  void setValue(uint8_t r, uint8_t g, uint8_t b);  // values between 0 and 255
  WbSFColor &operator=(const WbSFColor &other);
  bool operator==(const WbSFColor &other) const { return mValue == other.mValue; }

private:
  WbRgb mValue;
  void readSFColor(WbTokenizer *tokenizer, const QString &worldPath);
};

#endif
