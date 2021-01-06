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

#include "WbNodeReader.hpp"

#include "WbNode.hpp"
#include "WbNodeFactory.hpp"
#include "WbNodeModel.hpp"
#include "WbParser.hpp"
#include "WbProtoList.hpp"
#include "WbProtoModel.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"

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

// Since Qt 5.6.0, QStack::pop() gives a warning on Windows and recent versions of Ubuntu which we want to silence
// FIXME: these pragma should be removed when the problem is fixed in Qt
#ifndef __APPLE__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-overflow"
#endif
WbNodeReader::~WbNodeReader() {
  assert(!gCallStack.isEmpty());
  gCallStack.pop();
}
#ifndef __APPLE__
#pragma GCC diagnostic pop
#endif

WbNode *WbNodeReader::createNode(const QString &modelName, WbTokenizer *tokenizer, const QString &worldPath) {
  if (mMode == NORMAL)
    return WbNodeFactory::instance()->createNode(WbNodeModel::compatibleNodeName(modelName), tokenizer);

  WbNodeModel *const model = WbNodeModel::findModel(modelName);
  if (model)
    return new WbNode(modelName, worldPath, tokenizer);

  WbProtoModel *const proto = WbProtoList::current()->findModel(modelName, worldPath);
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
  WbNode *const node = createNode(WbNodeModel::compatibleNodeName(modelName), tokenizer, worldPath);
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

QList<WbNode *> WbNodeReader::readVrml(WbTokenizer *tokenizer, const QString &worldPath) {
  tokenizer->rewind();
  QList<WbNode *> nodes;
  while (!tokenizer->peekToken()->isEof()) {
    if (tokenizer->peekWord() == "PROTO")
      WbParser::skipProtoDefinition(tokenizer);
    else {
      WbNode *const node = readNode(tokenizer, worldPath);
      if (node)
        nodes.append(node);
    }
  }

  return nodes;
}

void WbNodeReader::addDefNode(WbNode *defNode) {
  mDefs.insert(defNode->defName(), defNode);
}

void WbNodeReader::removeDefNode(WbNode *defNode) {
  if (mDefs.value(defNode->defName()) == defNode)
    mDefs.remove(defNode->defName());
}
