// Copyright 1996-2019 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
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
#include "WbVrmlWriter.hpp"

#include <QtCore/QVector>

#include <cassert>

class WbMFString : public WbMultipleValue {
  Q_OBJECT

public:
  typedef WbMFIterator<WbMFString, QString> Iterator;

  WbMFString(WbTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  WbMFString(const WbMFString &other) { mVector = other.mVector; }
  virtual ~WbMFString() {}
  WbValue *clone() const override { return new WbMFString(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  int size() const override { return mVector.size(); }
  void clear() override;
  void writeItem(WbVrmlWriter &writer, int index) const override {
    assert(index >= 0 && index < size());
    writer.writeLiteralString(mVector[index]);
  }
  void insertDefaultItem(int index) override;
  WbVariant defaultNewVariant() const override { return WbVariant(QString()); }
  void removeItem(int index) override;
  WbVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return WbVariant(mVector[index]);
  }
  WbFieldType type() const override { return WB_MF_STRING; }
  const QString &item(int index) const {
    assert(index >= 0 && index < size());
    return mVector[index];
  }
  void setItem(int index, const QString &value);
  void addItem(const QString &value);
  void insertItem(int index, const QString &value);
  WbMFString &operator=(const WbMFString &other);
  bool operator==(const WbMFString &other) const { return mVector == other.mVector; }

protected:
  void readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) override;
  bool smallSeparator(int i) const override { return false; }

private:
  QVector<QString> mVector;
};

#endif
