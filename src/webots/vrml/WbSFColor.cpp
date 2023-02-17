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

#include "WbSFColor.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

void WbSFColor::readSFColor(WbTokenizer *tokenizer, const QString &worldPath) {
  try {
    const double r = tokenizer->nextToken()->toDouble();
    const double g = tokenizer->nextToken()->toDouble();
    const double b = tokenizer->nextToken()->toDouble();
    mValue.setValue(r, g, b);
    if (mValue.clampValuesIfNeeded())
      tokenizer->reportError(
        tr("Expected positive color values in range [0.0, 1.0], found [%1 %2 %3]. SFColor field reset to [%4 %5 %6]")
          .arg(r)
          .arg(g)
          .arg(b)
          .arg(mValue.red())
          .arg(mValue.green())
          .arg(mValue.blue()));
  } catch (...) {
    tokenizer->reportError(tr("Expected floating point value, found %1").arg(tokenizer->lastWord()), tokenizer->lastToken());
    tokenizer->ungetToken();  // unexpected token: keep the tokenizer coherent
    throw 0;                  // report the exception
  }
}

void WbSFColor::setValue(const WbRgb &c) {
  if (mValue == c)
    return;

  mValue = c;
  emit changed();
}

void WbSFColor::setValue(double r, double g, double b) {
  if (mValue == WbRgb(r, g, b))
    return;

  mValue.setValue(r, g, b);
  emit changed();
}

void WbSFColor::setValue(uint8_t r, uint8_t g, uint8_t b) {
  if (mValue == WbRgb(r, g, b))
    return;

  mValue.setValue(r, g, b);
  emit changed();
}

WbSFColor &WbSFColor::operator=(const WbSFColor &other) {
  if (mValue == other.mValue)
    return *this;

  mValue = other.mValue;
  emit changed();
  return *this;
}

bool WbSFColor::equals(const WbValue *other) const {
  const WbSFColor *that = dynamic_cast<const WbSFColor *>(other);
  return that && *this == *that;
}

void WbSFColor::copyFrom(const WbValue *other) {
  const WbSFColor *that = dynamic_cast<const WbSFColor *>(other);
  *this = *that;
}
