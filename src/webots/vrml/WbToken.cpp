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

#include "WbToken.hpp"

#include "WbProtoTemplateEngine.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QStringList>

static QStringList *gKeywords = NULL;

static void cleanup() {
  delete gKeywords;
}

static const QString NUMERIC_CHARS("+-0123456789.");

WbToken::WbToken(const QString &word, int line, int column) : mLine(line), mColumn(column), mWord(word) {
  const QChar w0 = mWord[0];  // shortcut

  if (word.startsWith('"') && word.endsWith('"'))
    mType = STRING;
  else if (word.startsWith(WbProtoTemplateEngine::openingToken()) && word.endsWith(WbProtoTemplateEngine::closingToken()))
    mType = TEMPLATE_STATEMENT;
  else if (NUMERIC_CHARS.contains(w0)) {
    // does this look like a double point number
    bool ok;
    // cppcheck-suppress ignoredReturnValue
    word.toDouble(&ok);
    mType = ok ? NUMERIC : INVALID;
  } else if (isKeyword(word))
    mType = KEYWORD;
  else if (isValidIdentifier(word))
    mType = IDENTIFIER;
  else if (word.length() == 1 && isPunctuation(w0))
    mType = PUNCTUATION;
  else
    mType = INVALID;
}

WbToken::WbToken(int line, int column) : mLine(line), mColumn(column), mWord("end of file"), mType(END) {
}

WbToken::WbToken(const WbToken &other) : mLine(other.mLine), mColumn(other.mColumn), mWord(other.mWord), mType(other.mType) {
}

float WbToken::toFloat() const {
  if (!isNumeric())
    throw 0;
  return mWord.toFloat();
}

double WbToken::toDouble() const {
  if (!isNumeric())
    throw 0;
  return mWord.toDouble();
}

int WbToken::toInt() const {
  if (!isNumeric())
    throw 0;
  return mWord.toInt();
}

bool WbToken::toBool() const {
  if (!isBoolean())
    throw 0;
  return mWord == "TRUE";
}

QString WbToken::toString() const {
  if (!isString())
    throw 0;

  QString s(mWord);
  s.remove(0, 1);  // remove leading "
  s.chop(1);       // remove trailing "
  return s;
}

void WbToken::setInt(int i) {
  mWord.setNum(i);
  mType = NUMERIC;
}

bool WbToken::isKeyword(const QString &word) {
  if (!gKeywords) {
    gKeywords = new QStringList();
    qAddPostRoutine(cleanup);

    // currently used by Webots:
    *gKeywords << "field"
               << "vrmlField"
               << "hiddenField"
               << "deprecatedField";
    *gKeywords << "DEF"
               << "USE"
               << "PROTO"
               << "EXTERNPROTO"
               << "IMPORTABLE";
    *gKeywords << "IS"
               << "TRUE"
               << "FALSE"
               << "NULL";
    *gKeywords << "MFBool"
               << "SFBool"
               << "SFColor"
               << "MFColor"
               << "SFFloat"
               << "MFFloat";
    *gKeywords << "SFInt32"
               << "MFInt32"
               << "SFNode"
               << "MFNode"
               << "SFRotation"
               << "MFRotation";
    *gKeywords << "SFString"
               << "MFString"
               << "SFVec2f"
               << "MFVec2f"
               << "SFVec3f"
               << "MFVec3f";

    // currently not used by Webots but reserved to enforce VRML interoperability:
    *gKeywords << "ROUTE"
               << "TO"
               << "eventIn"
               << "eventOut"
               << "exposedField";
    *gKeywords << "MFTime"
               << "SFImage"
               << "SFTime";
  };

  return gKeywords->indexOf(word) > -1;
}

bool WbToken::isValidIdentifierChar(const QChar &c, int pos) {
  // this is based on VRML'97 specs
  const ushort &v = c.unicode();
  if (v <= 0x20 || v == 0x22 || v == 0x23 || v == 0x27 || v == 0x2c || v == 0x2e || v == 0x5b || v == 0x5c || v == 0x5d ||
      v == 0x7b || v == 0x7d)
    return false;

  if (pos == 0 && v >= 0x30 && v <= 0x39)
    return false;

  return true;
}

bool WbToken::isValidIdentifier(const QString &token) {
  if (token.isEmpty())
    return false;

  int len = token.length();
  for (int i = 0; i < len; i++) {
    QChar c = token[i];
    if (!isValidIdentifierChar(c, i))
      return false;
  }

  return !isKeyword(token);
}

void WbToken::makeValidIdentifier(QString &id) {
  const int len = id.length();
  for (int i = 0; i < len; ++i)
    if (!isValidIdentifierChar(id[i], i))
      id[i] = '_';
}

bool WbToken::isPunctuation(QChar c) {
  static const QString Punctuation_CHARS("{}[]");
  return Punctuation_CHARS.contains(c);
}
