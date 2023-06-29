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

#include "WbMFVector2.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

void WbMFVector2::readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) {
  double x = tokenizer->nextToken()->toDouble();
  double y = tokenizer->nextToken()->toDouble();
  mVector.append(WbVector2(x, y));
}

void WbMFVector2::clear() {
  if (!mVector.empty()) {
    mVector.clear();
    emit changed();
    emit cleared();  // notify that all children have been removed
  }
}

void WbMFVector2::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, defaultNewVariant().toVector2());
  emit itemInserted(index);
  emit changed();
}

void WbMFVector2::removeItem(int index) {
  assert(index >= 0 && index < size());
  mVector.erase(mVector.begin() + index);
  emit itemRemoved(index);
  emit changed();
}

void WbMFVector2::setItem(int index, const WbVector2 &vec) {
  assert(index >= 0 && index < size());
  if (mVector[index] != vec) {
    mVector[index] = vec;
    emit itemChanged(index);
    emit changed();
  }
}

void WbMFVector2::addItem(const WbVector2 &vec) {
  mVector.append(vec);
  emit itemInserted(mVector.size() - 1);
  emit changed();
}

void WbMFVector2::insertItem(int index, const WbVector2 &vec) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, vec);
  emit itemInserted(index);
  emit changed();
}

void WbMFVector2::mult(double factor) {
  for (int i = 0, size = mVector.size(); i < size; ++i)
    mVector[i] *= factor;
}

WbMFVector2 &WbMFVector2::operator=(const WbMFVector2 &other) {
  if (mVector == other.mVector)
    return *this;

  mVector = other.mVector;
  emit changed();
  return *this;
}

bool WbMFVector2::equals(const WbValue *other) const {
  const WbMFVector2 *that = dynamic_cast<const WbMFVector2 *>(other);
  return that && *this == *that;
}

void WbMFVector2::copyFrom(const WbValue *other) {
  const WbMFVector2 *that = dynamic_cast<const WbMFVector2 *>(other);
  *this = *that;
}
