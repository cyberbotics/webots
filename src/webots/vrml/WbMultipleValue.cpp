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

#include "WbMultipleValue.hpp"

#include "WbSingleValue.hpp"
#include "WbTokenizer.hpp"
#include "WbWriter.hpp"

#include <cassert>

void WbMultipleValue::read(WbTokenizer *tokenizer, const QString &worldPath) {
  clear();

  if (tokenizer->peekWord() == "[") {
    tokenizer->skipToken("[");

    while (tokenizer->peekWord() != "]")
      readAndAddItem(tokenizer, worldPath);

    tokenizer->skipToken("]");
  } else
    readAndAddItem(tokenizer, worldPath);
}

void WbMultipleValue::write(WbWriter &writer) const {
  writer.writeMFStart();
  for (int i = 0; i < size(); i++) {
    writer.writeMFSeparator(i == 0, smallSeparator(i));
    writeItem(writer, i);
  }
  writer.writeMFEnd(size() == 0);
}

QString WbMultipleValue::toString(WbPrecision::Level level) const {
  QString result = "[ ";

  for (int i = 0; i < size(); i++)
    result += itemToString(i, level) + " ";

  result += "]";

  return result;
}

QString WbMultipleValue::itemToString(int index, WbPrecision::Level level) const {
  assert(index >= 0 && index < size());
  return variantValue(index).toSimplifiedStringRepresentation(level);
}

bool WbMultipleValue::valueAtIndexEqualsSingleValue(int index, const WbValue *other) const {
  assert(index >= 0 && index < size());
  const WbSingleValue *that = dynamic_cast<const WbSingleValue *>(other);
  return that && that->variantValue() == variantValue(index);
}
