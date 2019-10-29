// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "WbMFString.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

void WbMFString::readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) {
  mVector.append(tokenizer->nextToken()->toString());
}

void WbMFString::clear() {
  if (!mVector.empty()) {
    mVector.clear();
    emit changed();
  }
}

void WbMFString::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, defaultNewVariant().toString());
  emit itemInserted(index);
  emit changed();
}

void WbMFString::removeItem(int index) {
  assert(index >= 0 && index < size());
  mVector.erase(mVector.begin() + index);
  emit itemRemoved(index);
  emit changed();
}

void WbMFString::setItem(int index, const QString &value) {
  assert(index >= 0 && index < size());
  if (mVector[index] != value) {
    mVector[index] = value;
    emit itemChanged(index);
    emit changed();
  }
}

void WbMFString::addItem(const QString &value) {
  mVector.push_back(value);
  emit itemInserted(mVector.size() - 1);
  emit changed();
}

void WbMFString::insertItem(int index, const QString &value) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, value);
  emit itemInserted(index);
  emit changed();
}

WbMFString &WbMFString::operator=(const WbMFString &other) {
  if (mVector == other.mVector)
    return *this;

  mVector = other.mVector;
  emit changed();
  return *this;
}

bool WbMFString::equals(const WbValue *other) const {
  const WbMFString *that = dynamic_cast<const WbMFString *>(other);
  return that && *this == *that;
}

void WbMFString::copyFrom(const WbValue *other) {
  const WbMFString *that = dynamic_cast<const WbMFString *>(other);
  *this = *that;
}
