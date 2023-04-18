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

#ifndef WB_MF_DOUBLE_HPP
#define WB_MF_DOUBLE_HPP

//
// Description: field value that contains a multiple doubles
//

#include "WbMultipleValue.hpp"

#include "WbPrecision.hpp"
#include "WbWriter.hpp"

#include <QtCore/QVector>

#include <cassert>

class WbMFDouble : public WbMultipleValue {
  Q_OBJECT

public:
  typedef WbMFIterator<WbMFDouble, double> Iterator;

  WbMFDouble(WbTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  WbMFDouble(const WbMFDouble &other) : mVector(other.mVector) {}
  virtual ~WbMFDouble() {}
  WbValue *clone() const override { return new WbMFDouble(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  int size() const override { return mVector.size(); }
  void clear() override;
  void writeItem(WbWriter &writer, int index) const override {
    assert(index >= 0 && index < size());
    writer << itemToString(index, writer.isWebots() ? WbPrecision::DOUBLE_MAX : WbPrecision::FLOAT_MAX);
  }
  void insertDefaultItem(int index) override;
  WbVariant defaultNewVariant() const override { return WbVariant(0.0); }
  void removeItem(int index) override;
  WbVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return WbVariant(mVector[index]);
  }
  WbFieldType type() const override { return WB_MF_FLOAT; }
  const double &item(int index) const {
    assert(index >= 0 && index < size());
    return mVector[index];
  }
  void copyItemsTo(double values[], int max = -1) const;
  void findMinMax(double *min, double *max) const;
  void setItem(int index, double value);
  void setAllItems(const double *values);
  void multiplyAllItems(double factor);
  void addItem(double value);
  void insertItem(int index, double value);
  WbMFDouble &operator=(const WbMFDouble &other);
  bool operator==(const WbMFDouble &other) const { return mVector == other.mVector; }

protected:
  void readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) override;
  bool smallSeparator(int i) const override;

private:
  QVector<double> mVector;
};

#endif
