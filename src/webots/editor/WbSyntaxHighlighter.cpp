// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbSyntaxHighlighter.hpp"

#include "WbLanguage.hpp"
#include "WbTextBuffer.hpp"

#include <QtCore/QtAlgorithms>
#include <QtGui/QTextDocument>
#include <QtWidgets/QStyle>
#include <cassert>

static bool highlightedSectionComparator(WbSyntaxHighlighter::HighlightedSection left,
                                         WbSyntaxHighlighter::HighlightedSection right) {
  // compare by starting index
  return left.start < right.start;
}

WbSyntaxHighlighter *WbSyntaxHighlighter::createForLanguage(WbLanguage *language, QTextDocument *parent,
                                                            WbSyntaxHighlighter *previousHighlighter) {
  WbSyntaxHighlighter *highlighter = NULL;
  if (language == NULL)
    highlighter = new WbSyntaxHighlighter(parent);
  else {
    switch (language->code()) {
      case WbLanguage::C:
      case WbLanguage::CPP:
      case WbLanguage::JAVA:
        highlighter = new WbCHighlighter(language, parent);
        break;
      case WbLanguage::PYTHON:
        highlighter = new WbPythonHighlighter(language, parent);
        break;
      case WbLanguage::MATLAB:
      case WbLanguage::WBT:
      case WbLanguage::MAKEFILE:
        highlighter = new WbLanguageHighlighter(language, parent);
        break;
      case WbLanguage::PROTO:
        highlighter = new WbProtoHighlighter(language, parent);
        break;
      case WbLanguage::LUA:
        highlighter = new WbLuaHighlighter(language, parent);
        break;
      case WbLanguage::PLAIN_TEXT:
      default:
        highlighter = new WbSyntaxHighlighter(parent);
    }
  }

  if (previousHighlighter != NULL) {
    highlighter->setSearchTextRule(previousHighlighter->mSearchTextRule);
    delete previousHighlighter;
  }

  return highlighter;
}

WbSyntaxHighlighter::WbSyntaxHighlighter(QTextDocument *parent) : QSyntaxHighlighter(parent) {
  mSearchTextFormat.setBackground(Qt::gray);
}

void WbSyntaxHighlighter::setSearchTextRule(QRegExp regExp) {
  if (mSearchTextRule == regExp)
    return;

  mSearchTextRule = regExp;
  rehighlight();
}

void WbSyntaxHighlighter::highlightBlock(const QString &text) {
  highlightSearchText(text);
  setCurrentBlockState(0);
}

void WbSyntaxHighlighter::highlightSearchText(const QString &text, int offset) {
  if (mSearchTextRule.pattern().isEmpty())
    return;

  int index = mSearchTextRule.indexIn(text);
  while (index >= 0) {
    setFormat(index + offset, mSearchTextRule.matchedLength(), mSearchTextFormat);
    index = mSearchTextRule.indexIn(text, index + 1);
  }
}

WbLanguageHighlighter::WbLanguageHighlighter(const WbLanguage *language, QTextDocument *parentWidget) :
  WbSyntaxHighlighter(parentWidget) {
  // define styles
  WbTextBuffer *buffer = dynamic_cast<WbTextBuffer *>(parent()->parent()->parent());
  buffer->style()->polish(buffer);
  mApiFormat.setForeground(buffer->apiColor());
  mCommentFormat.setForeground(buffer->commentColor());
  mCommentFormat.setFontItalic(true);
  mKeywordFormat.setForeground(buffer->keywordColor());
  mNumberFormat.setForeground(buffer->numberColor());
  mPreprocessorFormat.setForeground(buffer->preprocessorColor());
  mQuotationFormat.setForeground(buffer->quotationColor());

  setDefaultRules(language, mHighlightingRules);
  mSingleCommentExpression = mHighlightingRules.last().pattern;
  mSingleCommentRuleIndex = mHighlightingRules.size() - 1;
}

void WbLanguageHighlighter::setDefaultRules(const WbLanguage *language, QVector<HighlightingRule> &highlightingRules) {
  // numbers
  HighlightingRule rule;
  rule.pattern = QRegExp("\\b[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?[fFlL]?\\b");
  rule.format = mNumberFormat;
  highlightingRules.append(rule);

  // add keywords and API words
  addWords(language->keywords(), mKeywordFormat, highlightingRules);
  addWords(language->apiWords(), mApiFormat, highlightingRules);

  // double quotes: "abc"
  rule.pattern = QRegExp("\"[^\"\\\\]*(?:\\\\.[^\"\\\\]*)*\"");
  rule.format = mQuotationFormat;
  highlightingRules.append(rule);

  // single quotes: 'a'
  rule.pattern = QRegExp("\'[^\']*\'");
  rule.format = mQuotationFormat;
  highlightingRules.append(rule);

  // single line comment
  rule.pattern = QRegExp(language->commentPrefix() + "[^\n]*");
  rule.format = mCommentFormat;
  highlightingRules.append(rule);
}

void WbLanguageHighlighter::highlightBlock(const QString &text) {
  highlightBlockSection(text, mHighlightingRules);
  setCurrentBlockState(0);
}

QList<WbSyntaxHighlighter::HighlightedSection> WbLanguageHighlighter::identifySectionsToHighlight(
  const QString &text, const QVector<HighlightingRule> &highlightingRules) {
  QList<HighlightedSection> sections;
  foreach (const HighlightingRule &rule, highlightingRules) {
    QRegExp expression(rule.pattern);
    int index = expression.indexIn(text);
    while (index >= 0) {
      sections.append(HighlightedSection(index, expression.matchedLength(), rule.format));
      index = expression.indexIn(text, index + 1);
    }
  }

  return sections;
}

void WbLanguageHighlighter::highlightBlockSection(const QString &text, const QVector<HighlightingRule> &highlightingRules,
                                                  int offset) {
  QList<HighlightedSection> formattedSections = identifySectionsToHighlight(text, highlightingRules);

  std::sort(formattedSections.begin(), formattedSections.end(), highlightedSectionComparator);

  int currentIndex = -1;
  foreach (HighlightedSection section, formattedSections) {
    if (currentIndex < section.start) {
      setFormat(section.start + offset, section.length, section.format);
      currentIndex = section.start + section.length;
    }
  }

  highlightSearchText(text, offset);
}

void WbLanguageHighlighter::addWords(const QStringList &words, const QTextCharFormat &format,
                                     QVector<HighlightingRule> &highlightingRules) {
  QStringListIterator it(words);
  while (it.hasNext()) {
    // empty rules cause an infinite loop in WbSyntaxHighlighter::highlightBlockSection
    assert(!it.peekNext().isEmpty());

    HighlightingRule rule;
    rule.pattern = QRegExp("\\b" + it.next() + "\\b");
    rule.format = format;
    highlightingRules.append(rule);
  }
}

WbMultiLineCommentHighlighter::WbMultiLineCommentHighlighter(const WbLanguage *language, const QRegExp &commentStartExpression,
                                                             const QRegExp &commentEndExpression, QTextDocument *parent) :
  WbLanguageHighlighter(language, parent),
  mCommentStartExpression(commentStartExpression),
  mCommentEndExpression(commentEndExpression),
  mCommentStartDelimiterLength(0) {
  // remove single line comment rule handled separately
  mHighlightingRules.remove(mSingleCommentRuleIndex);
}

void WbMultiLineCommentHighlighter::highlightBlock(const QString &text) {
  highlightBlockSection(text, 0);
}

void WbMultiLineCommentHighlighter::highlightBlockSection(const QString &text, int offset) {
  QList<HighlightedSection> formattedSections = identifySectionsToHighlight(text, mHighlightingRules);

  setCurrentBlockState(0);

  // identify single and multi line comments
  int startFormattingIndex = -1;
  int startIndex = 0;
  bool isFirstCommentLine = false;
  if (previousBlockState() != 1) {
    isFirstCommentLine = true;
    const int singleCommentIndex = mSingleCommentExpression.indexIn(text);
    startIndex = mCommentStartExpression.indexIn(text);
    if (singleCommentIndex >= 0 && (startIndex < 0 || singleCommentIndex < startIndex)) {
      // single comment
      formattedSections.prepend(HighlightedSection(singleCommentIndex, text.size() - singleCommentIndex, mCommentFormat));

      int singleCommentEnd = singleCommentIndex + mSingleCommentExpression.matchedLength();
      startIndex = mCommentStartExpression.indexIn(text, singleCommentEnd);
    }
  }

  while (startIndex >= 0) {
    int searchIndex = startIndex;
    if (isFirstCommentLine)
      searchIndex += mCommentStartDelimiterLength;
    int endIndex = mCommentEndExpression.indexIn(text, searchIndex);
    if (endIndex == -1) {
      setCurrentBlockState(1);
      formattedSections.prepend(HighlightedSection(startIndex, text.length() - startIndex, mCommentFormat));

      if (startIndex == 0 && previousBlockState() == 1) {
        // middle line of multi-line comment
        setFormat(offset, text.length(), mCommentFormat);
        return;
      }

      // break;
      startIndex = -1;
      continue;
    }

    int commentLength = endIndex - startIndex + mCommentEndExpression.matchedLength();
    if (commentLength > 0) {
      if (startIndex == 0 && previousBlockState() == 1) {
        startFormattingIndex = commentLength;
        setFormat(offset, commentLength, mCommentFormat);
      } else {
        formattedSections.prepend(HighlightedSection(startIndex, commentLength, mCommentFormat));
        // break;
        startIndex = -1;
        continue;
      }
    }

    int singleCommentIndex = mSingleCommentExpression.indexIn(text, startIndex + 1);
    startIndex = mCommentStartExpression.indexIn(text, startIndex + 1);
    if (singleCommentIndex >= 0 && (startIndex < 0 || singleCommentIndex < startIndex)) {
      // single comment
      formattedSections.prepend(HighlightedSection(singleCommentIndex, text.size() - singleCommentIndex, mCommentFormat));
      setFormat(singleCommentIndex + offset, text.size() - singleCommentIndex, mCommentFormat);

      // break;
      startIndex = -1;
      continue;
    }
  }

  std::sort(formattedSections.begin(), formattedSections.end(), highlightedSectionComparator);

  foreach (HighlightedSection section, formattedSections) {
    if (startFormattingIndex <= section.start) {
      setFormat(section.start + offset, section.length, section.format);
      startFormattingIndex = section.start + section.length;
    }
  }

  highlightSearchText(text, offset);
}

WbCHighlighter::WbCHighlighter(const WbLanguage *language, QTextDocument *parent) :
  WbMultiLineCommentHighlighter(language, QRegExp("/\\*"), QRegExp("\\*/"), parent) {
  // multiple line comments: /* abc */
  HighlightingRule rule;
  QStringListIterator it(language->preprocessorWords());
  while (it.hasNext()) {
    rule.pattern = QRegExp(it.next() + "[^\\n]*");
    rule.format = mPreprocessorFormat;
    // insert at position so that it has lower priority than single line comment
    mHighlightingRules.insert(mSingleCommentRuleIndex, rule);
  }

  mCommentStartDelimiterLength = 2;
}

WbLuaHighlighter::WbLuaHighlighter(const WbLanguage *language, QTextDocument *parent) :
  WbMultiLineCommentHighlighter(language, QRegExp("--\\[\\["), QRegExp("\\]\\]"), parent) {
  mCommentStartDelimiterLength = 4;
}

WbPythonHighlighter::WbPythonHighlighter(const WbLanguage *language, QTextDocument *parent) :
  WbMultiLineCommentHighlighter(language, QRegExp("\"\"\""), QRegExp("\"\"\""), parent) {
  mCommentStartDelimiterLength = 3;
}

WbProtoHighlighter::WbProtoHighlighter(const WbLanguage *language, QTextDocument *parent) :
  WbLuaHighlighter(WbLanguage::findByCode(WbLanguage::LUA), parent),
  mIsTemplateBlock(false),
  mTemplateStartPattern("%{"),
  mTemplateEndPattern("}%") {
  setDefaultRules(language, mProtoHighlightingRules);

  // template start and end patterns
  HighlightingRule rule;
  rule.pattern = QRegExp("%\\{");
  rule.format = mPreprocessorFormat;
  mTemplatePatternHighlightingRules.append(rule);
  rule.pattern = QRegExp("\\}%");
  rule.format = mPreprocessorFormat;
  mTemplatePatternHighlightingRules.append(rule);
}

void WbProtoHighlighter::highlightBlock(const QString &text) {
  WbLanguageHighlighter::highlightBlockSection(text, mTemplatePatternHighlightingRules, 0);
  setCurrentBlockState(0);
  if (previousBlockState() >= 0 && previousBlockState() < 2)
    mIsTemplateBlock = true;

  if (text.isEmpty() && previousBlockState() == 1) {
    setCurrentBlockState(1);
    return;
  }

  int blockOffset = 0;
  QString currentText(text);
  QString pattern;
  while (!currentText.isEmpty()) {
    pattern = mIsTemplateBlock ? mTemplateEndPattern : mTemplateStartPattern;
    const int patternIndex = currentText.indexOf(pattern);
    if (patternIndex < 0)
      break;
    else {
      if (mIsTemplateBlock)
        WbLuaHighlighter::highlightBlockSection(currentText.left(patternIndex), blockOffset);
      else
        WbLanguageHighlighter::highlightBlockSection(currentText.left(patternIndex), mProtoHighlightingRules, blockOffset);
      int offset = patternIndex + pattern.size();
      blockOffset += offset;
      currentText = currentText.right(currentText.size() - offset);
      mIsTemplateBlock = !mIsTemplateBlock;
    }
  }

  if (!currentText.isEmpty()) {
    if (mIsTemplateBlock)
      WbLuaHighlighter::highlightBlockSection(currentText, blockOffset);
    else
      WbLanguageHighlighter::highlightBlockSection(currentText, mProtoHighlightingRules, blockOffset);
  }

  if (!mIsTemplateBlock) {
    setCurrentBlockState(2);
  }

  WbSyntaxHighlighter::highlightSearchText(text, 0);
}
