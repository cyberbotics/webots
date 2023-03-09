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

#ifndef WB_SYNTAX_HIGHLIGHTER_HPP
#define WB_SYNTAX_HIGHLIGHTER_HPP

//
// Description: a class for highlighting the syntax of text documents
//

#include <QtCore/QRegularExpression>
#include <QtGui/QSyntaxHighlighter>
#include <QtGui/QTextCharFormat>

class QTextDocument;
class WbLanguage;

// a generic highlighter for search text
class WbSyntaxHighlighter : public QSyntaxHighlighter {
  Q_OBJECT

public:
  // create a highligher for a specific document and language
  static WbSyntaxHighlighter *createForLanguage(WbLanguage *language, QTextDocument *parent,
                                                WbSyntaxHighlighter *previousHighlighter = NULL);

  struct HighlightedSection {
    int start;
    int length;
    QTextCharFormat format;
    HighlightedSection(int start, int length, QTextCharFormat format) : start(start), length(length), format(format){};
  };

public slots:
  void setSearchTextRule(const QRegularExpression &regularExpression);

protected:
  explicit WbSyntaxHighlighter(QTextDocument *parent);
  void highlightSearchText(const QString &text, int offset = 0);
  void highlightBlock(const QString &text) override;

  QRegularExpression mSearchTextRule;
  QTextCharFormat mSearchTextFormat;
};

// a highlighter for programming, scripting or modelling languages
class WbLanguageHighlighter : public WbSyntaxHighlighter {
public:
  WbLanguageHighlighter(const WbLanguage *language, QTextDocument *parentWidget);

protected:
  struct HighlightingRule {
    QRegularExpression pattern;
    QTextCharFormat format;
  };

  void highlightBlock(const QString &text) override;
  void highlightBlockSection(const QString &text, const QVector<HighlightingRule> &highlightingRules, int offset = 0);
  QList<HighlightedSection> identifySectionsToHighlight(const QString &text,
                                                        const QVector<HighlightingRule> &highlightingRules);

  void setDefaultRules(const WbLanguage *language, QVector<HighlightingRule> &highlightingRules);
  void addWords(const QStringList &words, const QTextCharFormat &format, QVector<HighlightingRule> &highlightingRules);

  QVector<HighlightingRule> mHighlightingRules;
  QTextCharFormat mPreprocessorFormat;
  QTextCharFormat mKeywordFormat;
  QTextCharFormat mApiFormat;
  QTextCharFormat mCommentFormat;
  QTextCharFormat mQuotationFormat;
  QTextCharFormat mNumberFormat;
  QRegularExpression mSingleCommentExpression;
  int mSingleCommentRuleIndex;
};

// a highlighter specialized for languages with multiple line comments (/**/), e.g. C, C++ and Java
class WbMultiLineCommentHighlighter : public WbLanguageHighlighter {
public:
  WbMultiLineCommentHighlighter(const WbLanguage *language, const QRegularExpression &commentStartExpression,
                                const QRegularExpression &commentEndExpression, QTextDocument *parent);

protected:
  void highlightBlock(const QString &text) override;
  void highlightBlockSection(const QString &text, int offset);

  QRegularExpression mCommentStartExpression;
  QRegularExpression mCommentEndExpression;
  int mCommentStartDelimiterLength;
};

// a highlighter specialized C, C++ and Java
class WbCHighlighter : public WbMultiLineCommentHighlighter {
public:
  WbCHighlighter(const WbLanguage *language, QTextDocument *parent);
};

// a highlighter specialized for Python
class WbPythonHighlighter : public WbMultiLineCommentHighlighter {
public:
  WbPythonHighlighter(const WbLanguage *language, QTextDocument *parent);
};

// a highlighter specialized for Lua
class WbLuaHighlighter : public WbMultiLineCommentHighlighter {
public:
  WbLuaHighlighter(const WbLanguage *language, QTextDocument *parent);
};

// a highlighter specialized for PROTO files
class WbProtoHighlighter : public WbLuaHighlighter {
public:
  WbProtoHighlighter(const WbLanguage *language, QTextDocument *parent);

protected:
  void highlightBlock(const QString &text) override;
  void highlightBlockSection(const QString &text, int offset);

private:
  bool mIsTemplateBlock;
  QString mTemplateStartPattern;
  QString mTemplateEndPattern;
  QVector<HighlightingRule> mProtoHighlightingRules;
  QVector<HighlightingRule> mTemplatePatternHighlightingRules;
};

#endif
