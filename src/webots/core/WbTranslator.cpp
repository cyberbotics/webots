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

#include "WbTranslator.hpp"

#include "WbLog.hpp"
#include "WbPreferences.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QLocale>
#include <QtCore/QTranslator>

#include <cassert>

WbTranslator *WbTranslator::cInstance = NULL;

WbTranslator *WbTranslator::instance() {
  if (cInstance == NULL) {
    cInstance = new WbTranslator();
    qAddPostRoutine(WbTranslator::cleanup);
  }
  return cInstance;
}

void WbTranslator::cleanup() {
  delete cInstance;
  cInstance = NULL;
}

WbTranslator::WbTranslator() : mTranslator(NULL), mBasicTranslator(NULL) {
  updateSupportedLanguages();

  mTranslator = new QTranslator;
  mBasicTranslator = new QTranslator;

  WbPreferences *prefs = WbPreferences::instance();
  const QString &language = prefs->value("General/language").toString();

  if (!language.isEmpty()) {
    if (mSupportedLanguages.contains(language)) {
      if (!mTranslator->load("wb_" + language, WbStandardPaths::translationsPath()))
        WbLog::warning(QObject::tr("Cannot load translator for wb_%1.").arg(language));
      if (!mBasicTranslator->load("qt_" + language, WbStandardPaths::translationsPath()))
        WbLog::warning(QObject::tr("Cannot load translator for qt_%1.").arg(language));
    } else {
      WbLog::warning(
        QObject::tr("The language '%1' stored in preferences is not supported. Resetting to the language default (English).")
          .arg(language),
        true);
      prefs->setValue("General/language", "");
    }
  }
}

WbTranslator::~WbTranslator() {
  mSupportedLanguages.clear();
  delete mTranslator;
  delete mBasicTranslator;
}

void WbTranslator::updateSupportedLanguages() {
  mSupportedLanguages.clear();

  QStringList qmFilters;
  qmFilters << "wb_*.qm";

  QDir translationsDir = QDir(WbStandardPaths::translationsPath());

  mSupportedLanguages[""] = "English";

  const QFileInfoList &qmFileInfoList = translationsDir.entryInfoList(qmFilters, QDir::Files);
  foreach (const QFileInfo &fi, qmFileInfoList) {
    QString base = fi.baseName();  // e.g. wb_fr
    base.remove(0, 3);             // e.g. fr
    QLocale locale(base);
    // e.g. "French - français"
    QString content = QLocale::languageToString(locale.language()) + " - " + locale.nativeLanguageName();
    mSupportedLanguages[base] = content;
  }
}

QStringList WbTranslator::computeUserReadableLanguages() const {
  QStringList list;
  foreach (const QString &value, mSupportedLanguages)
    list << value;
  return list;
}

const QString &WbTranslator::findKey(const QString &language) const {
  QHashIterator<QString, QString> i(mSupportedLanguages);
  while (i.hasNext()) {
    i.next();
    if (i.value() == language)
      return i.key();
  }
  assert(0);
  return *(new QString);
}

int WbTranslator::findIndex(const QString &language) const {
  int counter = 0;
  QHashIterator<QString, QString> i(mSupportedLanguages);
  while (i.hasNext()) {
    i.next();
    if (i.key() == language)
      return counter;
    counter++;
  }
  assert(0);
  return 0;
}
