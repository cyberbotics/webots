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

#ifndef WB_MF_INT_HPP
#define WB_MF_INT_HPP

//
// Description: field value that contains a multiple integers
//

#include "WbMultipleValue.hpp"
#include "WbWriter.hpp"

#include <QtCore/QVector>

#include <cassert>

class WbMFInt : public WbMultipleValue {
  Q_OBJECT

public:
  typedef WbMFIterator<WbMFInt, int> Iterator;

  WbMFInt() {}
  WbMFInt(WbTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  WbMFInt(const WbMFInt &other) : mVector(other.mVector) {}
  virtual ~WbMFInt() {}
  WbValue *clone() const override { return new WbMFInt(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  int size() const override { return mVector.size(); }
  bool isEmpty() const { return mVector.empty(); }
  void clear() override;
  void writeItem(WbWriter &writer, int index) const override {
    assert(index >= 0 && index < size());
    writer << mVector[index];
  }
  void insertDefaultItem(int index) override;
  WbVariant defaultNewVariant() const override { return WbVariant(0); }
  void removeItem(int index) override;
  WbVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return WbVariant(mVector[index]);
  }
  WbFieldType type() const override { return WB_MF_INT32; }
  const int &item(int index) const {
    assert(index >= 0 && index < size());
    return mVector[index];
  }
  void setItem(int index, int value);
  void addItem(int value);
  void insertItem(int index, int value);
  void normalizeIndices();  // indices smaller than -1 are changed to -1
  WbMFInt &operator=(const WbMFInt &other);
  bool operator==(const WbMFInt &other) const { return mVector == other.mVector; }

protected:
  void readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) override;
  bool smallSeparator(int i) const override;

private:
  QVector<int> mVector;
};

#endif
