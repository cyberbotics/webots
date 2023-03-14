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

#ifndef WB_MF_VECTOR2_HPP
#define WB_MF_VECTOR2_HPP

//
// Description: field value that contains a multiple WbVector2
//

#include "WbMultipleValue.hpp"
#include "WbVector2.hpp"
#include "WbWriter.hpp"

#include <QtCore/QVector>

#include <cassert>

class WbMFVector2 : public WbMultipleValue {
  Q_OBJECT

public:
  typedef WbMFIterator<WbMFVector2, WbVector2> Iterator;

  WbMFVector2(WbTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  WbMFVector2(const WbMFVector2 &other) : mVector(other.mVector) {}
  virtual ~WbMFVector2() {}
  WbValue *clone() const override { return new WbMFVector2(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  int size() const override { return mVector.size(); }
  void clear() override;
  void writeItem(WbWriter &writer, int index) const override {
    assert(index >= 0 && index < size());
    writer << itemToString(index, writer.isWebots() ? WbPrecision::DOUBLE_MAX : WbPrecision::FLOAT_MAX);
  }
  void insertDefaultItem(int index) override;
  WbVariant defaultNewVariant() const override { return WbVariant(WbVector2()); }
  void removeItem(int index) override;
  WbVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return WbVariant(mVector[index]);
  }
  WbFieldType type() const override { return WB_MF_VEC2F; }
  const WbVector2 &item(int index) const {
    assert(index >= 0 && index < size());
    return mVector[index];
  }
  void setItem(int index, const WbVector2 &vec);
  void addItem(const WbVector2 &vec);
  void insertItem(int index, const WbVector2 &vec);
  void mult(double factor);
  WbMFVector2 &operator=(const WbMFVector2 &other);
  bool operator==(const WbMFVector2 &other) const { return mVector == other.mVector; }

protected:
  void readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) override;

private:
  QVector<WbVector2> mVector;
};

#endif
