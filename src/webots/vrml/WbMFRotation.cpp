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

#include "WbMFRotation.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

void WbMFRotation::readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) {
  double x = tokenizer->nextToken()->toDouble();
  double y = tokenizer->nextToken()->toDouble();
  double z = tokenizer->nextToken()->toDouble();
  double a = tokenizer->nextToken()->toDouble();
  mVector.append(WbRotation(x, y, z, a));
}

void WbMFRotation::clear() {
  if (!mVector.empty()) {
    mVector.clear();
    emit changed();
    emit cleared();  // notify that all children have been removed
  }
}

void WbMFRotation::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, defaultNewVariant().toRotation());
  emit itemInserted(index);
  emit changed();
}

void WbMFRotation::removeItem(int index) {
  assert(index >= 0 && index < size());
  mVector.erase(mVector.begin() + index);
  emit itemRemoved(index);
  emit changed();
}

void WbMFRotation::setItem(int index, const WbRotation &rot) {
  assert(index >= 0 && index < size());
  if (mVector[index] != rot) {
    mVector[index] = rot;
    emit itemChanged(index);
    emit changed();
  }
}

void WbMFRotation::addItem(const WbRotation &rot) {
  mVector.append(rot);
  emit itemInserted(mVector.size() - 1);
  emit changed();
}

void WbMFRotation::insertItem(int index, const WbRotation &rot) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, rot);
  emit itemInserted(index);
  emit changed();
}

WbMFRotation &WbMFRotation::operator=(const WbMFRotation &other) {
  if (mVector == other.mVector)
    return *this;

  mVector = other.mVector;
  emit changed();
  return *this;
}

bool WbMFRotation::equals(const WbValue *other) const {
  const WbMFRotation *that = dynamic_cast<const WbMFRotation *>(other);
  return that && *this == *that;
}

void WbMFRotation::copyFrom(const WbValue *other) {
  const WbMFRotation *that = dynamic_cast<const WbMFRotation *>(other);
  *this = *that;
}
