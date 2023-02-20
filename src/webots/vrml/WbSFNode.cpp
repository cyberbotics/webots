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

#include "WbSFNode.hpp"

#include "WbNode.hpp"
#include "WbNodeReader.hpp"
#include "WbTokenizer.hpp"
#include "WbWriter.hpp"

WbSFNode::WbSFNode(WbTokenizer *tokenizer, const QString &worldPath) {
  mValue = NULL;
  readSFNode(tokenizer, worldPath);
}

WbSFNode::WbSFNode(const WbSFNode &other) {
  if (other.mValue) {
    mValue = other.mValue->cloneAndReferenceProtoInstance();
    if (mValue)
      mValue->setInsertionCompleted();
  } else
    mValue = NULL;
}

void WbSFNode::readSFNode(WbTokenizer *tokenizer, const QString &worldPath) {
  delete mValue;

  if (WbNodeReader::current())
    // reading a regular list of nodes (file scope)
    mValue = WbNodeReader::current()->readNode(tokenizer, worldPath);
  else {
    // reading a default proto parameter (private scope)
    WbNodeReader reader;
    mValue = reader.readNode(tokenizer, worldPath);
  }
  if (mValue)
    mValue->setInsertionCompleted();
}

WbSFNode::WbSFNode(WbNode *node) {
  if (mValue == node)
    return;

  mValue = node;
  if (mValue)
    mValue->setInsertionCompleted();
}

WbSFNode::~WbSFNode() {
  delete mValue;
}

void WbSFNode::write(WbWriter &writer) const {
  if (mValue)
    mValue->write(writer);
  else if (!writer.isX3d() && !writer.isUrdf())
    writer << "NULL";
}

void WbSFNode::setValue(WbNode *node) {
  if (mValue == node)
    return;

  const WbNode *tmp = mValue;
  mValue = node;
  if (mValue)
    mValue->setInsertionCompleted();
  emit changed();
  delete tmp;
}

WbSFNode &WbSFNode::operator=(const WbSFNode &other) {
  if (this == &other)
    return *this;

  WbNode *tmp = mValue;
  if (other.mValue) {
    mValue = other.mValue->cloneAndReferenceProtoInstance();
    if (mValue)
      mValue->setInsertionCompleted();
  } else
    mValue = NULL;

  emit changed();
  delete tmp;
  return *this;
}

bool WbSFNode::operator==(const WbSFNode &other) const {
  if (this == &other)
    return true;

  if (mValue == NULL && other.mValue == NULL)
    return true;

  if (mValue == NULL || other.mValue == NULL)
    return false;

  return *mValue == *other.mValue;
}

bool WbSFNode::equals(const WbValue *other) const {
  const WbSFNode *that = dynamic_cast<const WbSFNode *>(other);
  return that && *this == *that;
}

void WbSFNode::copyFrom(const WbValue *other) {
  const WbSFNode *that = dynamic_cast<const WbSFNode *>(other);
  *this = *that;
}

void WbSFNode::defHasChanged() {
  if (mValue)
    mValue->defHasChanged();
}
