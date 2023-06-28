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

#include "WbMFDouble.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

void WbMFDouble::readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) {
  mVector.append(tokenizer->nextToken()->toDouble());
}

void WbMFDouble::clear() {
  if (mVector.size() > 0) {
    mVector.clear();
    emit changed();
    emit cleared();  // notify that all children have been removed
  }
}

void WbMFDouble::copyItemsTo(double values[], int max) const {
  if (max == -1 || max > mVector.size())
    max = mVector.size();

  memcpy(values, mVector.constData(), max * sizeof(double));
}

void WbMFDouble::findMinMax(double *min, double *max) const {
  if (mVector.isEmpty())
    return;

  *min = *max = mVector[0];
  foreach (double d, mVector) {
    *min = qMin(*min, d);
    *max = qMax(*max, d);
  }
}

void WbMFDouble::setItem(int index, double value) {
  assert(index >= 0 && index < size());
  if (mVector[index] != value) {
    mVector[index] = value;
    emit itemChanged(index);
    emit changed();
  }
}

void WbMFDouble::setAllItems(const double *values) {
  const int vectorSize = mVector.size();
  bool vectorHasChanged = false;
  for (int index = 0; index < vectorSize; index++) {
    if (mVector[index] != values[index]) {
      mVector[index] = values[index];
      emit itemChanged(index);
      vectorHasChanged = true;
    }
  }

  if (vectorHasChanged)
    emit changed();
}

void WbMFDouble::multiplyAllItems(double factor) {
  if (factor == 1.0)
    return;

  const int vectorSize = mVector.size();
  for (int index = 0; index < vectorSize; index++) {
    const double previousValue = mVector[index];
    if (previousValue != 0.0) {
      mVector[index] = factor * previousValue;
      emit itemChanged(index);
    }
  }

  emit changed();
}

void WbMFDouble::addItem(double value) {
  mVector.append(value);
  emit itemInserted(mVector.size() - 1);
  emit changed();
}

void WbMFDouble::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, defaultNewVariant().toDouble());
  emit itemInserted(index);
  emit changed();
}

void WbMFDouble::insertItem(int index, double value) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, value);
  emit itemInserted(index);
  emit changed();
}

void WbMFDouble::removeItem(int index) {
  assert(index >= 0 && index < size());
  mVector.erase(mVector.begin() + index);
  emit itemRemoved(index);
  emit changed();
}

WbMFDouble &WbMFDouble::operator=(const WbMFDouble &other) {
  if (mVector == other.mVector)
    return *this;

  mVector = other.mVector;
  emit changed();
  return *this;
}

bool WbMFDouble::equals(const WbValue *other) const {
  const WbMFDouble *that = dynamic_cast<const WbMFDouble *>(other);
  return that && *this == *that;
}

void WbMFDouble::copyFrom(const WbValue *other) {
  const WbMFDouble *that = dynamic_cast<const WbMFDouble *>(other);
  *this = *that;
}

bool WbMFDouble::smallSeparator(int i) const {
  return (i % 10 != 0 || WbMultipleValue::smallSeparator(i));  // 10 integers per line
}
