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

#ifndef WB_TOKEN_HPP
#define WB_TOKEN_HPP

//
// Description: atomic parsing element of a VRML text file
//

#include <QtCore/QString>

class WbToken {
public:
  // scan word and determine topen mType
  // token will be INVALID if the mType could not be determined
  WbToken(const QString &word, int line, int column);

  // create special END token
  WbToken(int line, int column);

  // copy
  WbToken(const WbToken &other);

  // the token STRING as found in the original text
  const QString &word() const { return mWord; }

  // line and column of the first character of the token
  int line() const { return mLine; }
  int column() const { return mColumn; }

  // test type
  bool isValid() const { return mType != INVALID; }
  bool isString() const { return mType == STRING; }
  bool isIdentifier() const { return mType == IDENTIFIER; }
  bool isKeyword() const { return mType == KEYWORD; }
  bool isNumeric() const { return mType == NUMERIC; }
  bool isPunctuation() const { return mType == PUNCTUATION; }
  bool isBoolean() const { return mType == KEYWORD && (mWord == "TRUE" || mWord == "FALSE"); }
  bool isTemplateStatement() const { return mType == TEMPLATE_STATEMENT; }
  bool isEof() const { return mType == END; }

  // for numeric tokens only: convert to float, double or int
  float toFloat() const;
  double toDouble() const;
  int toInt() const;

  // for boolean tokens only: conver to bool
  bool toBool() const;

  // for string tokens only: remove double quotes and replace escape sequences
  // e.g. "abc\\def" -> abc\efg
  QString toString() const;

  // needed to be able to correct parser errors for deprecated field types
  void setInt(int i);

  // test if argument is a valid VRML IDENTIFIER
  static bool isValidIdentifier(const QString &token);

  // replace illegal characters by with an underscore
  static void makeValidIdentifier(QString &id);

  // true for any VRML KEYWORD
  static bool isKeyword(const QString &word);

  // true for "{}[]" exclusively
  static bool isPunctuation(QChar c);

  // white space
  static bool isSpace(QChar c) { return c.isSpace() || c == ','; }

private:
  enum Type { INVALID, STRING, IDENTIFIER, KEYWORD, NUMERIC, PUNCTUATION, TEMPLATE_STATEMENT, END };

  int mLine, mColumn;
  QString mWord;
  Type mType;

  WbToken &operator=(const WbToken &);  // non copyable
  static bool isValidIdentifierChar(const QChar &c, int pos);
};

#endif
