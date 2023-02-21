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

#ifndef WB_SF_INT_HPP
#define WB_SF_INT_HPP

//
// Description: field value that contains a single integer
//

#include "WbSingleValue.hpp"
#include "WbWriter.hpp"

class WbSFInt : public WbSingleValue {
  Q_OBJECT

public:
  WbSFInt(WbTokenizer *tokenizer, const QString &worldPath) { readSFInt(tokenizer, worldPath); }
  WbSFInt(const WbSFInt &other);
  explicit WbSFInt(int value) : mValue(value) {}
  virtual ~WbSFInt() {}
  void read(WbTokenizer *tokenizer, const QString &worldPath) override { readSFInt(tokenizer, worldPath); }
  void write(WbWriter &writer) const override { writer << mValue; }
  WbValue *clone() const override { return new WbSFInt(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  WbVariant variantValue() const override { return WbVariant(mValue); }
  WbFieldType type() const override { return WB_SF_INT32; }
  int value() const { return mValue; }
  const int *valuePointer() const { return &mValue; }
  bool isZero() const { return mValue == 0; }
  void setValue(int i);
  void setValueNoSignal(int i) { mValue = i; }
  WbSFInt &operator=(const WbSFInt &other);
  bool operator==(const WbSFInt &other) const { return mValue == other.mValue; }

private:
  int mValue;
  void readSFInt(WbTokenizer *tokenizer, const QString &worldPath);
};

#endif
