// Copyright 1996-2025 Cyberbotics Ltd.
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

#include "WbNodeReader.hpp"

#include "WbNode.hpp"
#include "WbNodeFactory.hpp"
#include "WbNodeModel.hpp"
#include "WbParser.hpp"
#include "WbProtoManager.hpp"
#include "WbProtoModel.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"
#include "WbVrmlNodeUtilities.hpp"

#include <QDebug>  // For qDebug()
#include <QtCore/QStack>
#include <cassert>

static QStack<WbNodeReader *> gCallStack;

WbNodeReader *WbNodeReader::current() {
  if (gCallStack.isEmpty())
    return NULL;

  return gCallStack.top();
}

WbNodeReader::WbNodeReader(Mode mode) : mMode(mode), mIsReadingBoundingObject(false), mReadNodesCanceled(false) {
  gCallStack.push(this);
}

WbNodeReader::~WbNodeReader() {
  assert(!gCallStack.isEmpty());
  gCallStack.pop();
}

WbNode *WbNodeReader::createNode(const QString &modelName, WbTokenizer *tokenizer, const QString &worldPath,
                                 const QString &fileName) {
  // qDebug() << "WbNodeReader::createNode called with modelName:" << modelName
  //         << "worldPath:" << worldPath << "fileName:" << fileName;
  if (tokenizer) {
    const WbToken *currentToken = tokenizer->peekToken();
    if (currentToken) {
      //      qDebug() << "Tokenizer current token:" << currentToken->word() << "at line:" << currentToken->line() << "column:"
      //      << currentToken->column() << "pos:" << tokenizer->pos();
    } else {
      //     qDebug() << "Tokenizer: No current token available (peekToken() is NULL). Pos:" << tokenizer->pos();
    }
  } else {
    //   qDebug() << "WbNodeReader::createNode: tokenizer is NULL!";
  }

  if (mMode == NORMAL) {
    ///    qDebug() << "WbNodeReader::createNode: NORMAL mode, modelName:" << modelName;
    //  qDebug() << "WbNodeReader::createNode: NORMAL mode: Getting WbNodeFactory instance...";
    WbNodeFactory *factory = WbNodeFactory::instance();
    //   qDebug() << "WbNodeReader::createNode: Value of 'factory' variable after call to WbNodeFactory::instance():" <<
    //   factory;
    if (!factory) {
      //   qDebug() << "WbNodeReader::createNode: NORMAL mode: WbNodeFactory::instance() returned NULL!";
      // Potentially handle error or return NULL, though a segfault elsewhere suggests it's not just returning null
      return NULL;
    }
    // qDebug() << "WbNodeReader::createNode: NORMAL mode: WbNodeFactory instance obtained:" << factory;
    // qDebug() << "WbNodeReader::createNode: NORMAL mode: Calling WbNodeModel::compatibleNodeName for" << modelName;
    QString compatibleName = WbNodeModel::compatibleNodeName(modelName);
    // qDebug() << "WbNodeReader::createNode: NORMAL mode: Compatible name:" << compatibleName;
    /// qDebug() << "WbNodeReader::createNode: NORMAL mode: Calling factory->createNode for" << compatibleName;
    WbNode *node = factory->createNode(compatibleName, tokenizer);
    //  qDebug() << "WbNodeReader::createNode: NORMAL mode, created node:" << (node ? node->nodeModelName() : "NULL");
    return node;
  }

  if (modelName == "Transform") {
    //  qDebug() << "WbNodeReader::createNode: Handling Transform backward compatibility";
    bool isPose = WbVrmlNodeUtilities::transformBackwardCompatibility(tokenizer);
    // qDebug() << "WbNodeReader::createNode: isPose:" << isPose;
    return isPose ? new WbNode("Pose", worldPath, tokenizer) : new WbNode("Transform", worldPath, tokenizer);
  } else {
    if (WbNodeModel::findModel(modelName)) {
      // qDebug() << "WbNodeReader::createNode: Found model in WbNodeModel:" << modelName;
      WbNode *node = new WbNode(modelName, worldPath, tokenizer);
      // qDebug() << "WbNodeReader::createNode: Created WbNode for model:" << modelName << "node:" << (node ?
      // node->nodeModelName() : "NULL");
      return node;
    }
  }
  // qDebug() << "WbNodeReader::createNode: Attempting to find PROTO model:" << modelName << "in worldPath:" << worldPath <<
  // "fileName:" << fileName;
  WbProtoModel *const proto = WbProtoManager::instance()->findModel(modelName, worldPath, fileName);
  if (proto) {
    // qDebug() << "WbNodeReader::createNode: Found PROTO model:" << modelName << "proto name:" << proto->name();
    WbNode *node = WbNode::createProtoInstance(proto, tokenizer, worldPath);
    // qDebug() << "WbNodeReader::createNode: Created PROTO instance:" << (node ? node->nodeModelName() : "NULL");
    return node;
  }

  // qDebug() << "WbNodeReader::createNode: Skipped unknown node or PROTO:" << modelName;
  if (tokenizer)
    tokenizer->reportError(QObject::tr("Skipped unknown '%1' node or PROTO").arg(modelName));
  else
    // qDebug() << "WbNodeReader::createNode: Cannot report error, tokenizer is NULL for unknown model:" << modelName;
    return NULL;
  return NULL;
}

WbNode *WbNodeReader::readNode(WbTokenizer *tokenizer, const QString &worldPath) {
  if (tokenizer->peekWord() == "NULL") {
    tokenizer->skipToken("NULL");
    return NULL;
  }

  if (tokenizer->peekWord() == "USE") {
    tokenizer->skipToken("USE");
    const QString &useName = tokenizer->nextWord();

    // find USE name
    WbNode *const defNode = mDefs.value(useName, NULL);
    if (!defNode) {
      tokenizer->reportError(QObject::tr("Did not find a 'DEF %1' to match 'USE %1'").arg(useName));
      return NULL;
    }

    WbNode *const useNode = defNode->cloneDefNode();
    useNode->makeUseNode(defNode, true);
    return useNode;
  }

  QString defName;
  const WbNode *previousDefNode;
  if (tokenizer->peekWord() == "DEF") {
    tokenizer->skipToken("DEF");
    defName = tokenizer->nextWord();
    previousDefNode = mDefs.value(defName, NULL);
  } else
    previousDefNode = NULL;

  const QString &modelName = tokenizer->nextWord();
  const QString &parentFilePath = tokenizer->fileName().isEmpty() ? tokenizer->referralFile() : tokenizer->fileName();
  WbNode *const node = createNode(WbNodeModel::compatibleNodeName(modelName), tokenizer, worldPath, parentFilePath);
  if (!node) {
    if (tokenizer->lastWord() != "}")
      tokenizer->skipNode();
    return NULL;
  }

  if (!defName.isEmpty()) {
    node->setDefName(defName);
    if (previousDefNode == mDefs.value(defName, NULL))
      // check if descendant nodes have the same DEF name
      // if it is the case, then we should not overwrite the value
      addDefNode(node);
  }

  return node;
}

QList<WbNode *> WbNodeReader::readNodes(WbTokenizer *tokenizer, const QString &worldPath) {
  tokenizer->rewind();

  WbParser parser(tokenizer);
  while (tokenizer->peekWord() == "EXTERNPROTO" || tokenizer->peekWord() == "IMPORTABLE")  // consume EXTERNPROTO declarations
    parser.skipExternProto();

  QList<WbNode *> nodes;
  while (!tokenizer->peekToken()->isEof()) {
    emit readNodesHasProgressed(100 * tokenizer->pos() / tokenizer->totalTokensNumber());
    if (mReadNodesCanceled) {
      mReadNodesCanceled = false;
      return nodes;
    }
    // cppcheck-suppress constVariablePointer
    WbNode *node = readNode(tokenizer, worldPath);
    if (node)
      nodes.append(node);
  }

  return nodes;
}

void WbNodeReader::cancelReadNodes() {
  mReadNodesCanceled = true;
}

void WbNodeReader::addDefNode(WbNode *defNode) {
  mDefs.insert(defNode->defName(), defNode);
}

void WbNodeReader::removeDefNode(WbNode *defNode) {
  if (mDefs.value(defNode->defName()) == defNode)
    mDefs.remove(defNode->defName());
}
