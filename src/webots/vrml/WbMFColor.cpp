// Copyright 1996-2021 Cyberbotics Ltd.
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

#include "WbMFColor.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

void WbMFColor::readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) {
  const double r = tokenizer->nextToken()->toDouble();
  const double g = tokenizer->nextToken()->toDouble();
  const double b = tokenizer->nextToken()->toDouble();
  mVector.append(WbRgb(r, g, b));
}

void WbMFColor::clear() {
  if (!mVector.empty()) {
    mVector.clear();
    emit changed();
  }
}

void WbMFColor::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, defaultNewVariant().toColor());
  emit itemInserted(index);
  emit changed();
}

void WbMFColor::removeItem(int index) {
  assert(index >= 0 && index < size());
  mVector.erase(mVector.begin() + index);
  emit itemRemoved(index);
  emit changed();
}

void WbMFColor::setItem(int index, const WbRgb &value, bool signal) {
  assert(index >= 0 && index < size());
  if (mVector[index] != value) {
    mVector[index] = value;
    if (signal) {
      emit itemChanged(index);
      emit changed();
    }
  }
}

void WbMFColor::addItem(const WbRgb &value) {
  mVector.append(value);
  emit itemInserted(mVector.size() - 1);
  emit changed();
}

void WbMFColor::insertItem(int index, const WbRgb &value) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, value);
  emit itemInserted(index);
  emit changed();
}

WbMFColor &WbMFColor::operator=(const WbMFColor &other) {
  if (mVector == other.mVector)
    return *this;

  mVector = other.mVector;
  emit changed();
  return *this;
}

bool WbMFColor::equals(const WbValue *other) const {
  const WbMFColor *that = dynamic_cast<const WbMFColor *>(other);
  return that && *this == *that;
}

void WbMFColor::copyFrom(const WbValue *other) {
  const WbMFColor *that = dynamic_cast<const WbMFColor *>(other);
  *this = *that;
}
