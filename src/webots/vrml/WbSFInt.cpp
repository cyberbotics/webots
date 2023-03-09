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

#include "WbSFInt.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

WbSFInt::WbSFInt(const WbSFInt &other) {
  mValue = other.mValue;
}

void WbSFInt::readSFInt(WbTokenizer *tokenizer, const QString &worldPath) {
  try {
    mValue = tokenizer->nextToken()->toInt();
  } catch (...) {
    tokenizer->reportError(tr("Expected integer value, found %1").arg(tokenizer->lastWord()), tokenizer->lastToken());
    tokenizer->ungetToken();  // unexpected token: keep the tokenizer coherent
    throw 0;                  // report the exception
  }
}

void WbSFInt::setValue(int i) {
  if (mValue == i)
    return;

  mValue = i;
  emit changed();
}

WbSFInt &WbSFInt::operator=(const WbSFInt &other) {
  if (mValue == other.mValue)
    return *this;

  mValue = other.mValue;
  emit changed();
  return *this;
}

bool WbSFInt::equals(const WbValue *other) const {
  const WbSFInt *that = dynamic_cast<const WbSFInt *>(other);
  return that && *this == *that;
}

void WbSFInt::copyFrom(const WbValue *other) {
  const WbSFInt *that = dynamic_cast<const WbSFInt *>(other);
  *this = *that;
}
