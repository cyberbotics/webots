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

#ifndef WB_MF_STRING_HPP
#define WB_MF_STRING_HPP

//
// Description: field value that contains a multiple QString
//

#include "WbMultipleValue.hpp"
#include "WbWriter.hpp"

#include <QtCore/QStringList>

#include <cassert>

class WbMFString : public WbMultipleValue {
  Q_OBJECT

public:
  typedef WbMFIterator<WbMFString, QString> Iterator;

  WbMFString(WbTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  WbMFString(const WbMFString &other) : mValue(other.mValue) {}
  explicit WbMFString(const QStringList &value) : mValue(value) {}
  virtual ~WbMFString() {}
  WbValue *clone() const override { return new WbMFString(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  int size() const override { return mValue.size(); }
  void clear() override;
  void writeItem(WbWriter &writer, int index) const override {
    assert(index >= 0 && index < size());
    writer.writeLiteralString(mValue[index]);
  }
  void insertDefaultItem(int index) override;
  WbVariant defaultNewVariant() const override { return WbVariant(QString()); }
  void removeItem(int index) override;
  WbVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return WbVariant(mValue[index]);
  }
  const QStringList &value() const { return mValue; }
  WbFieldType type() const override { return WB_MF_STRING; }
  const QString &item(int index) const {
    assert(index >= 0 && index < size());
    return mValue[index];
  }
  void setValue(const QStringList &value);
  void setItem(int index, const QString &value);
  void addItem(const QString &value);
  void insertItem(int index, const QString &value);
  WbMFString &operator=(const WbMFString &other);
  bool operator==(const WbMFString &other) const { return mValue == other.mValue; }

protected:
  void readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) override;
  bool smallSeparator(int i) const override { return false; }

private:
  QStringList mValue;
};

#endif
