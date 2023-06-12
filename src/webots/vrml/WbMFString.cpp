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

#include "WbMFString.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

void WbMFString::readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) {
  mValue.append(tokenizer->nextToken()->toString());
}

void WbMFString::clear() {
  if (!mValue.empty()) {
    mValue.clear();
    emit changed();
    emit cleared();  // notify that all children have been removed
  }
}

void WbMFString::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mValue.insert(mValue.begin() + index, defaultNewVariant().toString());
  emit itemInserted(index);
  emit changed();
}

void WbMFString::removeItem(int index) {
  assert(index >= 0 && index < size());
  mValue.erase(mValue.begin() + index);
  emit itemRemoved(index);
  emit changed();
}

void WbMFString::setValue(const QStringList &value) {
  mValue = value;
  emit changed();
}

void WbMFString::setItem(int index, const QString &value) {
  assert(index >= 0 && index < size());
  if (mValue[index] != value) {
    mValue[index] = value;
    emit itemChanged(index);
    emit changed();
  }
}

void WbMFString::addItem(const QString &value) {
  mValue.push_back(value);
  emit itemInserted(mValue.size() - 1);
  emit changed();
}

void WbMFString::insertItem(int index, const QString &value) {
  assert(index >= 0 && index <= size());
  mValue.insert(mValue.begin() + index, value);
  emit itemInserted(index);
  emit changed();
}

WbMFString &WbMFString::operator=(const WbMFString &other) {
  if (mValue == other.mValue)
    return *this;

  mValue = other.mValue;
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
