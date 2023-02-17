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

#ifndef WB_MF_ROTATION_HPP
#define WB_MF_ROTATION_HPP

//
// Description: field value that contains a multiple WbRotation
//

#include "WbMultipleValue.hpp"
#include "WbRotation.hpp"
#include "WbWriter.hpp"

#include <QtCore/QVector>

#include <cassert>

class WbMFRotation : public WbMultipleValue {
  Q_OBJECT

public:
  typedef WbMFIterator<WbMFRotation, WbRotation> Iterator;

  WbMFRotation() {}
  WbMFRotation(WbTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  WbMFRotation(const WbMFRotation &other) : mVector(other.mVector) {}
  virtual ~WbMFRotation() {}
  WbValue *clone() const override { return new WbMFRotation(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  int size() const override { return mVector.size(); }
  void clear() override;
  void writeItem(WbWriter &writer, int index) const override {
    assert(index >= 0 && index < size());
    writer << itemToString(index, writer.isWebots() ? WbPrecision::DOUBLE_MAX : WbPrecision::FLOAT_MAX);
  }
  void insertDefaultItem(int index) override;
  WbVariant defaultNewVariant() const override { return WbVariant(WbRotation()); }
  void removeItem(int index) override;
  WbVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return WbVariant(mVector[index]);
  }
  WbFieldType type() const override { return WB_MF_ROTATION; }
  const WbRotation &item(int index) const {
    assert(index >= 0 && index < size());
    return mVector[index];
  }
  void setItem(int index, const WbRotation &rot);
  void addItem(const WbRotation &rot);
  void insertItem(int index, const WbRotation &rot);
  WbMFRotation &operator=(const WbMFRotation &other);
  bool operator==(const WbMFRotation &other) const { return mVector == other.mVector; }

protected:
  void readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) override;

private:
  QVector<WbRotation> mVector;
};

#endif
