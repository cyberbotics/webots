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

#ifndef WB_SF_BOOL_HPP
#define WB_SF_BOOL_HPP

//
// Description: field value that contains a single bool
//

#include "WbSingleValue.hpp"
#include "WbWriter.hpp"

class WbSFBool : public WbSingleValue {
  Q_OBJECT

public:
  WbSFBool(WbTokenizer *tokenizer, const QString &worldPath) { readSFBool(tokenizer, worldPath); }
  WbSFBool(const WbSFBool &other);
  explicit WbSFBool(bool value) : mValue(value) {}
  virtual ~WbSFBool() {}
  void read(WbTokenizer *tokenizer, const QString &worldPath) override { readSFBool(tokenizer, worldPath); }
  void write(WbWriter &writer) const override { writer << toString(WbPrecision::DOUBLE_MAX); }
  WbValue *clone() const override { return new WbSFBool(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  WbVariant variantValue() const override { return WbVariant(mValue); }
  WbFieldType type() const override { return WB_SF_BOOL; }
  bool value() const { return mValue; }
  const bool *valuePointer() const { return &mValue; }
  bool isTrue() const { return mValue; }
  bool isFalse() const { return !mValue; }
  void setValue(bool b);
  void setValueNoSignal(bool b) { mValue = b; }
  void setTrue() { setValue(true); }
  void setFalse() { setValue(false); }
  WbSFBool &operator=(const WbSFBool &other);
  bool operator==(const WbSFBool &other) const { return mValue == other.mValue; }

private:
  bool mValue;
  void readSFBool(WbTokenizer *tokenizer, const QString &worldPath);
};

#endif
