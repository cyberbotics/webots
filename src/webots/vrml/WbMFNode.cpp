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

#include "WbMFNode.hpp"
#include "WbNode.hpp"
#include "WbNodeReader.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"
#include "WbWriter.hpp"

#include <cassert>

WbMFNode::WbMFNode(const WbMFNode &other) {
  foreach (WbNode *const node, other.mVector) {
    WbNode *const copy = node->cloneAndReferenceProtoInstance();
    assert(copy);
    mVector.append(copy);
    copy->setInsertionCompleted();
  }
}

WbMFNode::~WbMFNode() {
  // qDeleteAll(mVector);  // Delete always USE nodes before DEF nodes
  const int n = mVector.size() - 1;
  for (int i = n; i >= 0; --i)
    delete mVector[i];
}

void WbMFNode::readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) {
  WbNode *node;
  if (WbNodeReader::current())
    // reading a regular list of nodes (file scope)
    node = WbNodeReader::current()->readNode(tokenizer, worldPath);
  else {
    // reading a default proto parameter (private scope)
    WbNodeReader reader(WbNodeReader::PROTO_MODEL);
    node = reader.readNode(tokenizer, worldPath);
  }
  if (node) {
    mVector.append(node);
    node->setInsertionCompleted();
  }
}

void WbMFNode::clear() {
  if (mVector.empty())
    return;

  QVector<WbNode *> tmp = mVector;
  mVector.clear();
  emit changed();

  // We don't want to use qDeleteAll(tmp) because we need to delete USE nodes before DEF nodes
  const int n = tmp.size() - 1;
  for (int i = n; i >= 0; --i)
    delete tmp[i];
}

void WbMFNode::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(index, defaultNewVariant().toNode());
  emit changed();
}

void WbMFNode::removeItem(int index) {
  assert(index >= 0 && index < size());
  WbNode *const tmp = mVector[index];
  mVector.remove(index);
  emit itemRemoved(index);
  emit changed();
  delete tmp;
}

bool WbMFNode::removeNode(WbNode *node) {
  const int index = mVector.indexOf(node);
  if (index == -1)
    return false;

  mVector.remove(index);
  emit itemRemoved(index);
  emit changed();
  return true;
}

void WbMFNode::setItem(int index, WbNode *node) {
  assert(index >= 0 && index < size());
  // NULL nodes are illegal in an MFNode
  assert(node);

  if (mVector[index] != node) {
    WbNode *const tmp = mVector[index];
    mVector[index] = node;
    node->setInsertionCompleted();
    emit itemChanged(index);
    emit changed();
    delete tmp;
  }
}

void WbMFNode::addItem(WbNode *node) {
  // NULL nodes are illegal in an MFNode
  assert(node);

  mVector.append(node);
  node->setInsertionCompleted();
  emit itemInserted(mVector.size() - 1);  // warning: inserted item may not be finalized
  emit changed();
}

void WbMFNode::insertItem(int index, WbNode *node) {
  assert(index >= 0 && index <= size());
  // NULL nodes are illegal in an MFNode
  assert(node);

  mVector.insert(index, node);
  node->setInsertionCompleted();
  emit itemInserted(index);
  emit changed();
}

WbMFNode &WbMFNode::operator=(const WbMFNode &other) {
  if (mVector == other.mVector)
    return *this;

  while (mVector.size() > 0)
    removeItem(0);

  const int m = other.mVector.size();
  for (int i = 0; i < m; ++i) {
    WbNode *const copy = other.mVector[i]->cloneAndReferenceProtoInstance();
    assert(copy);  // test clone() function
    addItem(copy);
  }

  emit changed();
  return *this;
}

bool WbMFNode::operator==(const WbMFNode &other) const {
  if (this == &other)
    return true;

  if (size() != other.size())
    return false;

  const int vectorSize = mVector.size();
  for (int i = 0; i < vectorSize; ++i) {
    const WbNode *const n1 = mVector[i];
    const WbNode *const n2 = other.mVector[i];
    if (*n1 != *n2)
      return false;
  }

  return true;
}

bool WbMFNode::equals(const WbValue *other) const {
  const WbMFNode *that = dynamic_cast<const WbMFNode *>(other);
  return that && *this == *that;
}

void WbMFNode::copyFrom(const WbValue *other) {
  const WbMFNode *that = dynamic_cast<const WbMFNode *>(other);
  *this = *that;
}

void WbMFNode::writeItem(WbWriter &writer, int index) const {
  assert(index >= 0 && index < size());
  mVector[index]->write(writer);
}

void WbMFNode::defHasChanged() {
  const int vectorSize = mVector.size();
  for (int i = 0; i < vectorSize; ++i)
    mVector[i]->defHasChanged();
}

int WbMFNode::nodeIndex(const WbNode *node) const {
  if (mVector.contains(const_cast<WbNode *>(node)))
    return mVector.indexOf(const_cast<WbNode *>(node));
  return -1;
}

void WbMFNode::write(WbWriter &writer) const {
  writer.writeMFStart();
  int c = 0;
  const int vectorSize = mVector.size();
  for (int i = 0; i < vectorSize; ++i) {
    if (writer.isWebots() || writer.isUrdf() || mVector[i]->shallExport()) {
      if (!writer.isX3d())
        writer.writeMFSeparator(c == 0, smallSeparator(i));
      writeItem(writer, i);
      ++c;
    }
  }
  writer.writeMFEnd(c == 0);
}
