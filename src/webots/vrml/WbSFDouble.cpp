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

#include "WbSFDouble.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

#include <cmath>

void WbSFDouble::readSFDouble(WbTokenizer *tokenizer, const QString &worldPath) {
  try {
    mValue = tokenizer->nextToken()->toDouble();
  } catch (...) {
    tokenizer->reportError(tr("Expected floating point value, found %1").arg(tokenizer->lastWord()), tokenizer->lastToken());
    tokenizer->ungetToken();  // unexpected token: keep the tokenizer coherent
    throw 0;                  // report the exception
  }
}

void WbSFDouble::makeAbsolute() {
  if (mValue >= 0.0)
    return;

  mValue = ::fabs(mValue);
  emit changed();
}

bool WbSFDouble::clip(double min, double max) {
  if (mValue < min) {
    mValue = min;
    emit changed();
    return true;
  }

  if (mValue > max) {
    mValue = max;
    emit changed();
    return true;
  }

  return false;
}

void WbSFDouble::setValue(double d) {
  if (mValue == d)
    return;

  mValue = d;
  emit changed();
}

void WbSFDouble::add(double d) {
  if (d == 0.0)
    return;

  mValue += d;
  emit changed();
}

void WbSFDouble::mult(double factor) {
  if (factor == 1.0)
    return;

  mValue *= factor;
  emit changed();
}

WbSFDouble &WbSFDouble::operator=(const WbSFDouble &other) {
  if (mValue == other.mValue)
    return *this;

  mValue = other.mValue;
  emit changed();
  return *this;
}

bool WbSFDouble::equals(const WbValue *other) const {
  const WbSFDouble *that = dynamic_cast<const WbSFDouble *>(other);
  return that && *this == *that;
}

void WbSFDouble::copyFrom(const WbValue *other) {
  const WbSFDouble *that = dynamic_cast<const WbSFDouble *>(other);
  *this = *that;
}
