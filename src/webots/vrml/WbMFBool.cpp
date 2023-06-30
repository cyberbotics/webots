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

#include "WbMFBool.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

void WbMFBool::readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) {
  mVector.append(tokenizer->nextToken()->toBool());
}

void WbMFBool::clear() {
  if (!mVector.empty()) {
    mVector.clear();
    emit changed();
    emit cleared();  // notify that all children have been removed
  }
}

void WbMFBool::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, defaultNewVariant().toBool());
  emit itemInserted(index);
  emit changed();
}

void WbMFBool::removeItem(int index) {
  assert(index >= 0 && index < size());
  mVector.erase(mVector.begin() + index);
  emit itemRemoved(index);
  emit changed();
}

void WbMFBool::setItem(int index, bool b) {
  assert(index >= 0 && index < size());
  if (mVector[index] != b) {
    mVector[index] = b;
    emit itemChanged(index);
    emit changed();
  }
}

void WbMFBool::addItem(const bool &b) {
  mVector.append(b);
  emit itemInserted(mVector.size() - 1);
  emit changed();
}

void WbMFBool::insertItem(int index, const bool &b) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, b);
  emit itemInserted(index);
  emit changed();
}

WbMFBool &WbMFBool::operator=(const WbMFBool &other) {
  if (mVector == other.mVector)
    return *this;

  mVector = other.mVector;
  emit changed();
  return *this;
}

bool WbMFBool::equals(const WbValue *other) const {
  const WbMFBool *that = dynamic_cast<const WbMFBool *>(other);
  return that && *this == *that;
}

void WbMFBool::copyFrom(const WbValue *other) {
  const WbMFBool *that = dynamic_cast<const WbMFBool *>(other);
  *this = *that;
}
