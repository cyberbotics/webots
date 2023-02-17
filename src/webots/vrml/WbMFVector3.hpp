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

#ifndef WB_MF_VECTOR3_HPP
#define WB_MF_VECTOR3_HPP

//
// Description: field value that contains a multiple WbVector3
//

#include "WbMultipleValue.hpp"
#include "WbVector3.hpp"
#include "WbWriter.hpp"

#include <QtCore/QVector>

#include <cassert>

class WbMFVector3 : public WbMultipleValue {
  Q_OBJECT

public:
  typedef WbMFIterator<WbMFVector3, WbVector3> Iterator;

  WbMFVector3() {}
  WbMFVector3(WbTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  WbMFVector3(const WbMFVector3 &other) : mVector(other.mVector) {}
  virtual ~WbMFVector3() {}
  WbValue *clone() const override { return new WbMFVector3(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  int size() const override { return mVector.size(); }
  void clear() override;
  void writeItem(WbWriter &writer, int index) const override {
    assert(index >= 0 && index < size());
    writer << itemToString(index, writer.isWebots() ? WbPrecision::DOUBLE_MAX : WbPrecision::FLOAT_MAX);
  }
  void insertDefaultItem(int index) override;
  WbVariant defaultNewVariant() const override { return WbVariant(WbVector3()); }
  void removeItem(int index) override;
  WbVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return WbVariant(mVector[index]);
  }
  WbFieldType type() const override { return WB_MF_VEC3F; }
  const WbVector3 &item(int index) const {
    assert(index >= 0 && index < size());
    return mVector[index];
  }
  void setItem(int index, const WbVector3 &vec);
  void rescale(const WbVector3 &scale);
  void rescaleAndTranslate(int coordinate, double scale, double translation);
  void rescaleAndTranslate(const WbVector3 &scale, const WbVector3 &translation);
  void translate(int coordinate, double translation);
  void translate(const WbVector3 &translation);
  void addItem(const WbVector3 &vec);
  void insertItem(int index, const WbVector3 &vec);
  void mult(double factor);
  WbMFVector3 &operator=(const WbMFVector3 &other);
  bool operator==(const WbMFVector3 &other) const { return mVector == other.mVector; }

protected:
  void readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) override;

private:
  QVector<WbVector3> mVector;
};

#endif
