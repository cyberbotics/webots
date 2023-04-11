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

#ifndef WB_TOKENIZER_HPP
#define WB_TOKENIZER_HPP

//
// WbTokenizer: provides the functionality to split a text file into VRML-based tokens
//

#include "WbVersion.hpp"

#include <QtCore/QChar>
#include <QtCore/QString>
#include <QtCore/QVector>

class QTextStream;
class WbToken;

class WbTokenizer {
public:
  WbTokenizer();
  ~WbTokenizer();

  // files types: .wbt, .proto or .wrl
  enum FileType { UNKNOWN, WORLD, PROTO, MODEL };
  FileType fileType() const { return mFileType; }

  // build list of tokens
  // returns the number of invalid tokens found
  int tokenize(const QString &fileName, const QString &prefix = QString());
  int tokenizeString(const QString &string);
  const QString &fileName() const { return mFileName; }

  // returns the info stored as (#) comments in the file header
  const QString &info() const { return mInfo; }

  // returns the tags stored as (# tags: tag1, tag2) comments in the file header
  const QStringList tags() const;

  // returns the scripting language used in procedural PROTOs stored as (# templateEngine: string)
  const QString templateLanguage() const;

  // returns the license stored as (# license: string) comments in the file header
  const QString license() const;

  // returns the license URL stored as (# license url: string) comments in the file header
  const QString licenseUrl() const;

  // returns the documentation URL stored as (# license url: string) comments in the file header
  const QString documentationUrl() const;

  // returns file version found in file header
  const WbVersion &fileVersion() const { return mFileVersion; }

  // static method returning the current world file version
  static const WbVersion &worldFileVersion();

  // returns true if peekToken() or nextToken() would be valid
  bool hasMoreTokens() const { return mIndex < mVector.size(); }

  // returns total number of tokens
  int totalTokensNumber() const { return mVector.size(); }

  // look at next token/word without consuming it
  WbToken *peekToken() const { return mVector.at(mIndex); }
  const QString &peekWord() const;

  // skip a known token
  // in debug version the expected word is checked against the effective token word
  // in release version this is equivalent to: (void)nextToken()
  void skipToken(const char *expectedWord);

  // return next token and consume it
  WbToken *nextToken() { return mVector[mIndex++]; }
  const QString &nextWord();

  // returns the last token returned by nextToken()/nextWord()
  WbToken *lastToken() const;
  const QString &lastWord() const;

  // control position in token stream
  void rewind() { mIndex = 0; }
  void forward() { mIndex = mVector.size(); }
  void ungetToken() { --mIndex; }
  int pos() const { return mIndex; }
  void seek(int pos) { mIndex = pos; }

  // fuzzy skip an unknown VRML node
  // skips all the fields and values until right after the node's closing brace "}"
  // if deleteTokens is true, then all the tokens of the node are deleted
  void skipNode(bool deleteTokens = false);

  // fuzzy skip an unknown VRML value
  // skips the values values until the next field is reached
  // if deleteTokens is true then all the tokens of the field, including the field name, are deleted
  void skipField(bool deleteTokens = false);

  // report an error to WbLog about the last token consumed with nextToken() or nextWord()
  // if a token is specified: report error about the specified token,
  // otherwise report an error on the last token consumed
  // output format: filename:line:column: message
  void reportError(const QString &message, const WbToken *token = NULL) const;

  // reports an general error on a file without specifying the token
  void reportFileError(const QString &message) const;

  int setErrorOffset(int offset) { return mErrorOffset = offset; }
  void setReferralFile(const QString &file) { mReferralFile = file; }
  const QString &referralFile() const { return mReferralFile; }

private:
  QString mFileName;
  FileType mFileType;
  WbVersion mFileVersion;
  QString mInfo;
  QVector<WbToken *> mVector;
  QTextStream *mStream;
  QString mLineString;
  QChar mChar;
  int mLine, mColumn, mTokenLine, mTokenColumn;
  int mIndex;
  bool mAtEnd;
  QString mReferralFile;
  int mErrorOffset;

  QString readLine();
  QChar readChar();
  QString readWord();
  void skipWhiteSpace();
  bool checkFileHeader();
  bool readFileInfo(bool headerRequired, bool displayWarning, const QString &headerTag, bool isProto = false);
  static void displayHeaderHelp(const QString &fileName, const QString &headerTag);
  void markTokenStart();
  static FileType fileTypeFromFileName(const QString &fileName);
  void reportError(const QString &message, int line, int column) const;
};

#endif
