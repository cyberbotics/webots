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

#ifndef WB_MF_COLOR_HPP
#define WB_MF_COLOR_HPP

//
// Description: field value that contains a multiple WbRgb
//

#include "WbMultipleValue.hpp"
#include "WbRgb.hpp"
#include "WbWriter.hpp"

#include <QtCore/QVector>

#include <cassert>

class WbMFColor : public WbMultipleValue {
  Q_OBJECT

public:
  typedef WbMFIterator<WbMFColor, WbRgb> Iterator;

  WbMFColor(WbTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  WbMFColor(const WbMFColor &other) : mVector(other.mVector) {}
  virtual ~WbMFColor() {}
  WbValue *clone() const override { return new WbMFColor(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  int size() const override { return mVector.size(); }
  void clear() override;
  void writeItem(WbWriter &writer, int index) const override {
    assert(index >= 0 && index < size());
    writer << itemToString(index, writer.isWebots() ? WbPrecision::DOUBLE_MAX : WbPrecision::FLOAT_MAX);
  }
  void insertDefaultItem(int index) override;
  WbVariant defaultNewVariant() const override { return WbVariant(WbRgb()); }
  void removeItem(int index) override;
  WbVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return WbVariant(mVector[index]);
  }
  WbFieldType type() const override { return WB_MF_COLOR; }
  const WbRgb &item(int index) const {
    assert(index >= 0 && index < size());
    return mVector[index];
  }
  void setItem(int index, const WbRgb &value, bool signal = true);
  void addItem(const WbRgb &value);
  void insertItem(int index, const WbRgb &value);
  WbMFColor &operator=(const WbMFColor &other);
  bool operator==(const WbMFColor &other) const { return mVector == other.mVector; }
  bool operator!=(const WbMFColor &other) const { return mVector != other.mVector; }

protected:
  void readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) override;

private:
  QVector<WbRgb> mVector;
};

#endif
