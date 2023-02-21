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

#include "WbSFString.hpp"

#include "WbToken.hpp"
#include "WbTokenizer.hpp"

void WbSFString::readSFString(WbTokenizer *tokenizer, const QString &worldPath) {
  try {
    mValue = tokenizer->nextToken()->toString();
  } catch (...) {
    tokenizer->reportError(tr("Expected string value, found %1").arg(tokenizer->lastWord()), tokenizer->lastToken());
    tokenizer->ungetToken();  // unexpected token: keep the tokenizer coherent
    throw 0;                  // report the exception
  }
}

void WbSFString::setValue(const QString &s) {
  mValue = s;
  emit changed();
}

WbSFString &WbSFString::operator=(const WbSFString &other) {
  if (mValue == other.mValue)
    return *this;

  mValue = other.mValue;
  emit changed();
  return *this;
}

bool WbSFString::equals(const WbValue *other) const {
  const WbSFString *that = dynamic_cast<const WbSFString *>(other);
  return that && *this == *that;
}

void WbSFString::copyFrom(const WbValue *other) {
  const WbSFString *that = dynamic_cast<const WbSFString *>(other);
  *this = *that;
}
