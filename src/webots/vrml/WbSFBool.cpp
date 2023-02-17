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

#include "WbSFBool.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

WbSFBool::WbSFBool(const WbSFBool &other) {
  mValue = other.mValue;
}

void WbSFBool::readSFBool(WbTokenizer *tokenizer, const QString &worldPath) {
  try {
    mValue = tokenizer->nextToken()->toBool();
  } catch (...) {
    tokenizer->reportError(tr("Expected boolean value, found %1").arg(tokenizer->lastWord()), tokenizer->lastToken());
    tokenizer->ungetToken();  // unexpected token: keep the tokenizer coherent
    throw 0;                  // report the exception
  }
}

void WbSFBool::setValue(bool b) {
  if (mValue == b)
    return;

  mValue = b;
  emit changed();
}

WbSFBool &WbSFBool::operator=(const WbSFBool &other) {
  if (mValue == other.mValue)
    return *this;

  mValue = other.mValue;
  emit changed();
  return *this;
}

bool WbSFBool::equals(const WbValue *other) const {
  const WbSFBool *that = dynamic_cast<const WbSFBool *>(other);
  return that && *this == *that;
}

void WbSFBool::copyFrom(const WbValue *other) {
  const WbSFBool *that = dynamic_cast<const WbSFBool *>(other);
  *this = *that;
}
