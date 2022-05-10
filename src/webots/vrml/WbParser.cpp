// Copyright 1996-2022 Cyberbotics Ltd.
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

#include "WbParser.hpp"

#include "../core/WbStandardPaths.hpp"
#include "../nodes/utils/WbDownloader.hpp"
#include "WbApplicationInfo.hpp"
#include "WbFieldModel.hpp"
#include "WbLog.hpp"
#include "WbNodeModel.hpp"
#include "WbProtoList.hpp"
#include "WbProtoModel.hpp"
#include "WbProtoTemplateEngine.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>

#include <cassert>

static double cLegacyGravity = 9.81;

double WbParser::legacyGravity() {
  return cLegacyGravity;
}

WbParser::WbParser(WbTokenizer *tokenizer) : mTokenizer(tokenizer), mMode(NONE), mDownloader(NULL) {
}

const QString &WbParser::fileName() const {
  return mTokenizer->fileName();
}

WbParser::~WbParser() {
  delete mDownloader;
  mDownloader = NULL;
}

void WbParser::parseDoubles(int n) {
  for (int i = 0; i < n; ++i) {
    if (!nextToken()->isNumeric())
      reportUnexpected(QObject::tr("floating point value"));
  }
}

const QString WbParser::parseUrl() {
  if (!peekToken()->isString())
    reportUnexpected(QObject::tr("string literal"));

  const QString url = nextToken()->toString();
  if (!(url.toLower().startsWith("webots://") || url.toLower().startsWith("https://"))) {
    mTokenizer->reportError(QObject::tr("Expected url starting with 'webots://' or 'https://'"));
    throw 0;
  }

  if (!url.toLower().endsWith(".proto")) {
    mTokenizer->reportError(QObject::tr("Expected url to end with '.proto' or '.PROTO'"));
    throw 0;
  }

  return url;
}

void WbParser::parseInt() {
  // this is a backwards compatibility fix for pre-8.6 files, where
  // 'filtering' could have boolean values (it now takes integral values)
  QString fieldName(mTokenizer->lastWord());
  WbToken *fieldValue = nextToken();

  if (fieldValue->isBoolean() && fieldName == "filtering" && mTokenizer->fileVersion() < WbVersion(8, 6, 0)) {
    WbLog::warning(QObject::tr("Boolean values for 'ImageTexture.filtering' field are deprecated"
                               " from Webots 8.6 onwards; the value has been converted automatically."
                               " Please update your PROTO and world files accordingly."),
                   false, WbLog::PARSING);

    bool isFilteringOn = fieldValue->toBool();
    if (isFilteringOn)
      fieldValue->setInt(4);
    else
      fieldValue->setInt(0);
  } else if (!fieldValue->isNumeric())
    reportUnexpected(QObject::tr("integer value"));
}

void WbParser::parseBool() {
  if (!nextToken()->isBoolean())
    reportUnexpected(QObject::tr("boolean value"));
}

void WbParser::parseString() {
  if (!nextToken()->isString())
    reportUnexpected(QObject::tr("string literal"));
}

void WbParser::parseSingleFieldValue(WbFieldType type, const QString &worldPath) {
  switch (type) {
    case WB_SF_INT32:
      parseInt();
      break;
    case WB_SF_FLOAT:
      parseDoubles(1);
      break;
    case WB_SF_STRING:
      parseString();
      break;
    case WB_SF_VEC2F:
      parseDoubles(2);
      break;
    case WB_SF_VEC3F:
    case WB_SF_COLOR:
      parseDoubles(3);
      break;
    case WB_SF_ROTATION:
      parseDoubles(4);
      break;
    case WB_SF_BOOL:
      parseBool();
      break;
    case WB_SF_NODE: {
      if (peekWord() == "NULL")
        skipToken();
      else
        parseNode(worldPath);
      break;
    }
    default:
      assert(false);
  }
}

void WbParser::parseFieldValue(WbFieldType type, const QString &worldPath) {
  if (WbValue::isSingle(type))
    parseSingleFieldValue(type, worldPath);
  else {
    const WbFieldType singleType = WbValue::toSingle(type);
    if (peekWord() == "[") {
      skipToken();
      while (peekWord() != "]")
        parseSingleFieldValue(singleType, worldPath);
      skipToken();
    } else
      parseSingleFieldValue(singleType, worldPath);
  }
}

void WbParser::parseFieldAcceptedValues(WbFieldType type, const QString &worldPath) {
  while (nextWord() != '}') {
    mTokenizer->ungetToken();
    WbParser::parseSingleFieldValue(type, worldPath);
  }
}

void WbParser::parseFieldDeclaration(const QString &worldPath) {
  WbToken *const token = nextToken();
  if (token->word() != "field" && token->word() != "vrmlField" && token->word() != "hiddenField" &&
      token->word() != "deprecatedField" && token->word() != "unconnectedField")
    reportUnexpected(QObject::tr("'field', 'unconnectedField', 'vrmlField' or 'hiddenField' keywords"));

  // check field type
  const WbFieldType type = WbValue::vrmlNameToType(nextWord());
  if (!type)
    reportUnexpected(QObject::tr("field type name"));

  if (nextWord() == "{")
    parseFieldAcceptedValues((WbFieldType)(type & (~WB_MF)), worldPath);
  else
    mTokenizer->ungetToken();
  parseIdentifier();
  parseFieldValue(type, worldPath);
}

void WbParser::reportError(const QString &message, const WbToken *token) const {
  mTokenizer->reportError(message, token);
}

void WbParser::reportFileError(const QString &message) const {
  mTokenizer->reportFileError(message);
}

void WbParser::reportUnexpected(const QString &expected) const {
  const WbToken *const found = mTokenizer->lastToken();
  QString foundWord(found->word());
  if (foundWord.split(QRegularExpression("\\s+")).size() == 1)
    foundWord = QString("'%1'").arg(foundWord);

  QString expectedWord(expected);
  if (expectedWord.split(QRegularExpression("\\s+")).size() == 1)
    expectedWord = QString("'%1'").arg(expectedWord);

  mTokenizer->reportError(QObject::tr("Expected %1, found %2").arg(expectedWord, foundWord), found);
  throw 0;
}

bool WbParser::parseWorld(const QString &worldPath) {
  mTokenizer->rewind();
  mMode = WBT;
  try {
    while (!peekToken()->isEof()) {
      while (peekWord() == "EXTERNPROTO")  // consume all EXTERNPROTO tokens, they are retrieved separately
        skipExternProto();

      parseNode(worldPath);
    }
  } catch (...) {
    return false;
  }
  return true;
}

// parse VRML file syntax
// there can be in-line PROTO definitions, in this case they are
// also parsed and added to the current WbProtoList
bool WbParser::parseVrml(const QString &worldPath) {
  mTokenizer->rewind();
  mMode = VRML;
  try {
    while (mTokenizer->hasMoreTokens() && !peekToken()->isEof())
      // proto statements can appear in between node statements
      // this is important for VRML import
      if (peekWord() == "PROTO") {
        mMode = PROTO;
        const int pos = mTokenizer->pos();
        parseProtoDefinition(worldPath);
        mTokenizer->seek(pos);
        WbProtoList::current()->readModel(mTokenizer, worldPath);
        mMode = VRML;
      } else
        parseNode(worldPath);
  } catch (...) {
    return false;
  }
  return true;
}

void WbParser::parseProtoDefinition(const QString &worldPath) {
  parseProtoInterface(worldPath);

  parseExactWord("{");
  parseNode(worldPath);
  parseExactWord("}");
}

bool WbParser::parseObject(const QString &worldPath) {
  mTokenizer->rewind();
  mMode = WBO;
  try {
    parseNode(worldPath);
  } catch (...) {
    return false;
  }
  return true;
}

bool WbParser::parseNodeModel() {
  mTokenizer->rewind();
  mMode = WRL;
  try {
    parseIdentifier();  // node name
    parseExactWord("{");

    while (peekWord() != "}")
      parseFieldDeclaration("");

    skipToken();  // "}"
  } catch (...) {
    return false;
  }
  return true;
}

WbToken *WbParser::nextToken() {
  return mTokenizer->nextToken();
}

const QString &WbParser::nextWord() {
  return mTokenizer->nextToken()->word();
}

const QString &WbParser::peekWord() const {
  return mTokenizer->peekToken()->word();
}

WbToken *WbParser::peekToken() {
  return mTokenizer->peekToken();
}

void WbParser::skipToken() {
  assert(mTokenizer->hasMoreTokens());
  mTokenizer->nextToken();
}

void WbParser::parseEof() {
  if (nextToken()->isEof())
    return;

  reportUnexpected(QObject::tr("end of file"));
}

const QString &WbParser::parseIdentifier(const QString &expected) {
  WbToken *token = nextToken();

  if (!token->isIdentifier())
    reportUnexpected(expected);

  return token->word();
}

void WbParser::parseExactWord(const QString &word) {
  if (nextToken()->word() == word)
    return;

  reportUnexpected(word);
}

void WbParser::parseNode(const QString &worldPath) {
  // printf("parseNode (next word: %s / mode %d)\n", peekWord().toUtf8().constData(), mMode);

  // TODO: does calling parseExternProto here cover all cases ? (proto and world)

  if (peekWord() == "USE") {
    skipToken();
    parseIdentifier();
    return;
  }

  if (peekWord() == "DEF") {
    skipToken();
    parseIdentifier();
  }

  QString nodeName = parseIdentifier(QObject::tr("node or PROTO name"));
  const QString &newNodeName = WbNodeModel::compatibleNodeName(nodeName);
  if (newNodeName != nodeName) {
    reportError(QObject::tr("Deprecated '%1', please use '%2' node instead").arg(nodeName).arg(newNodeName));
    nodeName = newNodeName;
  }

  const WbNodeModel *const nodeModel = WbNodeModel::findModel(nodeName);
  if (nodeModel) {
    parseExactWord("{");
    while (peekWord() != "}")
      parseField(nodeModel, worldPath);
    skipToken();  // "}";
    // if no coordinate system was explicitly set in parseField(), set the default value.
    if (nodeModel->name() == "WorldInfo" && WbProtoTemplateEngine::coordinateSystem().isEmpty()) {
      if (mTokenizer->fileVersion() < WbVersion(2020, 1, 0))  // earlier than R2020b
        WbProtoTemplateEngine::setCoordinateSystem("NUE");
      else
        WbProtoTemplateEngine::setCoordinateSystem("ENU");
    }
    return;
  }

  // printf(" > node is a proto\n");
  const WbProtoModel *const protoModel = WbProtoList::current()->customFindModel(nodeName, worldPath);
  if (protoModel) {
    parseExactWord("{");
    while (peekWord() != "}")
      parseParameter(protoModel, worldPath);
    skipToken();  // "}";
    return;
  }

  reportError(QObject::tr("Skipped unknown '%1' node or PROTO").arg(nodeName));
  mTokenizer->skipNode();
}

void WbParser::parseField(const WbNodeModel *nodeModel, const QString &worldPath) {
  const QString &fieldName = parseIdentifier(QObject::tr("field name or '}'"));
  // we need to set the coordinate system to the WbProtoTemplateEngine early enough to be able to pass the "coordinate_system"
  // as a context dictionary to procedural PROTO parameter nodes that are created before the WorldInfo node.
  if (nodeModel->name() == "WorldInfo") {
    if (mTokenizer->fileVersion() >= WbVersion(2020, 1, 0) && fieldName == "coordinateSystem") {
      QString coordinateSystem = peekWord();
      if (coordinateSystem.at(0) == '"' && coordinateSystem.back() == '"') {
        coordinateSystem = coordinateSystem.mid(1, coordinateSystem.size() - 2);
        WbProtoTemplateEngine::setCoordinateSystem(coordinateSystem);
      }
    } else if (mTokenizer->fileVersion() < WbVersion(2020, 1, 0) && fieldName == "gravity") {
      const double x = nextWord().toDouble();
      const double y = nextWord().toDouble();
      const double z = peekWord().toDouble();
      cLegacyGravity = sqrt(x * x + y * y + z * z);
      reportError(QObject::tr("Found deprecated gravity vector (%1 %2 %3) in WorldInfo, using gravity vector length: %4")
                    .arg(x)
                    .arg(y)
                    .arg(z)
                    .arg(cLegacyGravity));
      mTokenizer->skipField(true);
      return;
    }
  }
  const WbFieldModel *const fieldModel = nodeModel->findFieldModel(fieldName);
  if (!fieldModel) {
    reportError(QObject::tr("Skipped unknown '%1' field in %2 node").arg(fieldName, nodeModel->name()));
    mTokenizer->skipField(true);
    return;
  }

  if (peekWord() == "IS") {
    skipToken();
    if (mMode != PROTO)
      reportError(QObject::tr("'IS' keyword is only allowed in .proto files"));

    parseIdentifier(QObject::tr("PROTO field name"));
    return;
  }

  parseFieldValue(fieldModel->type(), worldPath);
}

void WbParser::parseParameter(const WbProtoModel *protoModel, const QString &worldPath) {
  const QString &parameterName = parseIdentifier(QObject::tr("parameter name or '}'"));

  // no check on Webots-generated hidden fields
  if (parameterName == "hidden") {
    mTokenizer->nextToken();
    mTokenizer->skipField();
    return;
  }

  const WbFieldModel *const parameterModel = protoModel->findFieldModel(parameterName);
  if (!parameterModel) {
    reportError(QObject::tr("Skipped unknown '%1' parameter in %2 PROTO").arg(parameterName, protoModel->name()));
    mTokenizer->skipField(true);
    return;
  }

  // allow to pass a PROTO parameter as value of a sub PROTO node parameter
  if (peekWord() == "IS") {
    skipToken();

    parseIdentifier(QObject::tr("PROTO field name"));
    return;
  }

  parseFieldValue(parameterModel->type(), worldPath);
}

bool WbParser::parseProtoInterface(const QString &worldPath) {
  mMode = PROTO;
  try {
    parseExactWord("PROTO");
    parseIdentifier();
    parseExactWord("[");

    while (peekWord() != "]" && mTokenizer->hasMoreTokens())
      parseFieldDeclaration(worldPath);

    parseExactWord("]");

  } catch (...) {
    return false;
  }
  return true;
}

bool WbParser::parseProtoBody(const QString &worldPath) {
  mMode = PROTO;
  try {
    parseNode(worldPath);
    parseEof();
  } catch (...) {
    return false;
  }
  return true;
}

void WbParser::skipProtoDefinition(WbTokenizer *tokenizer) {
  // we should skip instead of parsing the tokens
  // but this is used only for VRML import, and can be optimized later
  WbParser parser(tokenizer);
  parser.mMode = PROTO;
  parser.parseProtoDefinition("");
}

void WbParser::skipExternProto() {
  mTokenizer->skipToken("EXTERNPROTO");

  const WbToken *token = nextToken();
  if (!token->isString())
    reportUnexpected("string literal");
}
