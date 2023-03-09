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

/*
 * Description:  Class defining a simple tokenizer
 */

#ifndef TOKENIZER_HPP
#define TOKENIZER_HPP

#include <QtCore/QChar>
#include <QtCore/QString>

class Tokenizer {
public:
  explicit Tokenizer(const QString &string, const QChar &separator = ';');
  virtual ~Tokenizer() {}

  QString nextToken();
  bool hasMoreToken() const;
  QString remainingString() const;

private:
  QString mString;
  QChar mSeparator;
  int mCurrentPos;
};

#endif
