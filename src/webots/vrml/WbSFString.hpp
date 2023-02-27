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

#ifndef WB_SF_STRING_HPP
#define WB_SF_STRING_HPP

//
// Description: field value that contains a single QString
//

#include "WbSingleValue.hpp"
#include "WbWriter.hpp"

class WbSFString : public WbSingleValue {
  Q_OBJECT

public:
  explicit WbSFString(const QString &s) : mValue(s) {}
  WbSFString(WbTokenizer *tokenizer, const QString &worldPath) { readSFString(tokenizer, worldPath); }
  WbSFString(const WbSFString &other) : mValue(other.mValue) {}
  virtual ~WbSFString() {}
  void read(WbTokenizer *tokenizer, const QString &worldPath) override { readSFString(tokenizer, worldPath); }
  void write(WbWriter &writer) const override { writer.writeLiteralString(mValue); }
  WbValue *clone() const override { return new WbSFString(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  WbVariant variantValue() const override { return WbVariant(mValue); }
  WbFieldType type() const override { return WB_SF_STRING; }
  const QString &value() const { return mValue; }
  bool isEmpty() const { return mValue.isEmpty(); }
  void clear() { return mValue.clear(); }
  void setValue(const QString &s);
  WbSFString &operator=(const WbSFString &other);
  bool operator==(const WbSFString &other) const { return mValue == other.mValue; }

private:
  QString mValue;
  void readSFString(WbTokenizer *tokenizer, const QString &worldPath);
};

#endif
