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

#include "WbMFInt.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

void WbMFInt::readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) {
  mVector.append(tokenizer->nextToken()->toInt());
}

void WbMFInt::clear() {
  if (mVector.size() > 0) {
    mVector.clear();
    emit changed();
    emit cleared();  // notify that all children have been removed
  }
}

void WbMFInt::setItem(int index, int value) {
  assert(index >= 0 && index < size());
  if (mVector[index] != value) {
    mVector[index] = value;
    emit itemChanged(index);
    emit changed();
  }
}

void WbMFInt::addItem(int value) {
  mVector.append(value);
  emit itemInserted(mVector.size() - 1);
  emit changed();
}

void WbMFInt::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(index, defaultNewVariant().toInt());
  emit itemInserted(index);
  emit changed();
}

void WbMFInt::insertItem(int index, int value) {
  assert(index >= 0 && index <= size());
  mVector.insert(index, value);
  emit itemInserted(index);
  emit changed();
}

void WbMFInt::removeItem(int index) {
  assert(index >= 0 && index < size());
  mVector.remove(index);
  emit itemRemoved(index);
  emit changed();
}

void WbMFInt::normalizeIndices() {
  bool modified = false;
  for (QVector<int>::iterator i = mVector.begin(); i != mVector.end(); ++i)
    if (*i < -1) {
      *i = -1;
      modified = true;
    }

  if (modified)
    emit changed();
}

WbMFInt &WbMFInt::operator=(const WbMFInt &other) {
  if (mVector == other.mVector)
    return *this;

  mVector = other.mVector;
  emit changed();
  return *this;
}

bool WbMFInt::equals(const WbValue *other) const {
  const WbMFInt *that = dynamic_cast<const WbMFInt *>(other);
  return that && *this == *that;
}

void WbMFInt::copyFrom(const WbValue *other) {
  const WbMFInt *that = dynamic_cast<const WbMFInt *>(other);
  *this = *that;
}

bool WbMFInt::smallSeparator(int i) const {
  return (i % 10 != 0 || WbMultipleValue::smallSeparator(i));  // 10 integers per line
}
