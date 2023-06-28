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
  if (mMode == NORMAL)
    return WbNodeFactory::instance()->createNode(WbNodeModel::compatibleNodeName(modelName), tokenizer);

  if (modelName == "Transform")
    return WbVrmlNodeUtilities::transformBackwardCompatibility(tokenizer) ? new WbNode("Pose", worldPath, tokenizer) :
                                                                            new WbNode("Transform", worldPath, tokenizer);
  else {
    if (WbNodeModel::findModel(modelName))
      return new WbNode(modelName, worldPath, tokenizer);
  }
  WbProtoModel *const proto = WbProtoManager::instance()->findModel(modelName, worldPath, fileName);
  if (proto)
    return WbNode::createProtoInstance(proto, tokenizer, worldPath);

  tokenizer->reportError(QObject::tr("Skipped unknown '%1' node or PROTO").arg(modelName));
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
