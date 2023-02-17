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

#include "WbSFVector2.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

void WbSFVector2::readSFVector2(WbTokenizer *tokenizer, const QString &worldPath) {
  try {
    double xCoordinate = tokenizer->nextToken()->toDouble();
    double yCoordinate = tokenizer->nextToken()->toDouble();
    mValue.setXy(xCoordinate, yCoordinate);
    mValue.clamp();
  } catch (...) {
    tokenizer->reportError(tr("Expected floating point value, found %1").arg(tokenizer->lastWord()), tokenizer->lastToken());
    tokenizer->ungetToken();  // unexpected token: keep the tokenizer coherent
    throw 0;                  // report the exception
  }
}

void WbSFVector2::setValue(const WbVector2 &v) {
  if (mValue == v)
    return;

  mValue = v;
  emit changed();
}

void WbSFVector2::setValue(double x, double y) {
  if (mValue == WbVector2(x, y))
    return;

  mValue.setXy(x, y);
  emit changed();
}

void WbSFVector2::setValue(const double xy[2]) {
  if (mValue == WbVector2(xy))
    return;

  mValue.setXy(xy);
  emit changed();
}

void WbSFVector2::setValueFromWebots(const WbVector2 &v) {
  if (mValue == v)
    return;

  mValue = v;
  emit changedByWebots();
}

void WbSFVector2::mult(double factor) {
  if (factor == 1.0)
    return;

  mValue *= factor;
  emit changed();
}

void WbSFVector2::setComponent(int index, double d) {
  if (component(index) == d)
    return;

  mValue[index] = d;
  emit changed();
}

WbSFVector2 &WbSFVector2::operator=(const WbSFVector2 &other) {
  if (mValue == other.mValue)
    return *this;

  mValue = other.mValue;
  emit changed();
  return *this;
}

bool WbSFVector2::equals(const WbValue *other) const {
  const WbSFVector2 *that = dynamic_cast<const WbSFVector2 *>(other);
  return that && *this == *that;
}

void WbSFVector2::copyFrom(const WbValue *other) {
  const WbSFVector2 *that = dynamic_cast<const WbSFVector2 *>(other);
  *this = *that;
}
