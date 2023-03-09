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

#ifndef WB_LANGUAGE_HPP
#define WB_LANGUAGE_HPP

//
// Description: definition of the languages that can be edited in Webots text editor
//

#include <QtCore/QString>
#include <QtCore/QStringList>

class WbLanguage {
public:
  // find language by looking at the filename's suffix
  static WbLanguage *findByFileName(const QString &fileName);

  // get the list of possible executable extensions, e.g. "", ".exe", ".class", ...
  static const QStringList &exectuableExtensions();

  // return true if extension is ".so"
  static bool isUnixLibraryExtension(const QString &extension);

  // get the list of possible file extensions
  static const QStringList &sourceFileExtensions();  // ".c", ".cpp", ".c++", ".py", ".java", ".m", etc.
  static const QStringList &headerFileExtensions();  // ".h", ".hh", ".hpp", ".h++", ".hxx", etc.
  static const QStringList &dataFileExtensions();    // ".xml", ".ini", ".csv" ".txt", ".motion", etc.

  // language enum codes
  enum { PLAIN_TEXT, C, CPP, JAVA, PYTHON, MATLAB, MAKEFILE, WBT, PROTO, LUA, MAX };
  int code() const { return mCode; }
  static WbLanguage *findByCode(int code);

  // language name, e.g. "C++", "Java" ...
  const QString &name() const { return mName; }

  // file suffix to use when generating new source files
  const QString &defaultFileSuffix() const { return mDefaultFileSuffix; }

  // comment start, e.g. "//", "%", "#" ...
  const QString &commentPrefix() const { return mCommentPrefix; }

  // true for C, C++ and Java languages
  bool isCompilable() const { return mIsCompilable; }

  // file extension to use when compiling source files
  const QString &compilationExtension() const;

  // language keywords, e.g. "for", "if" ...
  const QStringList &keywords() const { return mKeywords; }

  // Webots API words, e.g. "Accelerometer", "wb_accelerometer_enable" ...
  const QStringList &apiWords() const { return mApiWords; }

  // preprocessor words for C and C++, e.g. "#assert", "#define" ...
  const QStringList &preprocessorWords() const { return mPreprocessorWords; }

  // all words that need autocompletion
  QStringList autoCompletionWords() const { return mApiWords + mKeywords + mPreprocessorWords; }

private:
  int mCode;
  QString mName;
  QString mDefaultFileSuffix;
  QString mCommentPrefix;
  bool mIsCompilable;
  QStringList mKeywords;
  QStringList mApiWords;
  QStringList mPreprocessorWords;

  WbLanguage(int code, const QString &name, const QString &defaultFileSuffix, const QString &commentPrefix, bool isCompilable);
  ~WbLanguage();
  void addKeywords(const QString &words);
  void addApiWords(const QString &words);
  void addPreprocessorWords(const QString &words);

  static void cleanup();
};

#endif
