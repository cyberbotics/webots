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

#ifndef WB_TRANSLATOR_HPP
#define WB_TRANSLATOR_HPP

#include <QtCore/QHash>
#include <QtCore/QString>
#include <QtCore/QStringList>

class QTranslator;

class WbTranslator {
public:
  static WbTranslator *instance();

  QTranslator *translator() const { return mTranslator; }
  QTranslator *basicTranslator() const { return mBasicTranslator; }

  QStringList computeUserReadableLanguages() const;       // "French - Francais", ...
  const QString &findKey(const QString &language) const;  // "French - Francais" -> "fr"
  int findIndex(const QString &language) const;           // "fr" -> 2

private:
  static void cleanup();

  static WbTranslator *cInstance;

  WbTranslator();
  WbTranslator(const WbTranslator &);             // non construction-copyable
  WbTranslator &operator=(const WbTranslator &);  // non copyable
  ~WbTranslator();

  void updateSupportedLanguages();

  QHash<QString, QString> mSupportedLanguages;
  QTranslator *mTranslator;
  QTranslator *mBasicTranslator;
};

#endif
