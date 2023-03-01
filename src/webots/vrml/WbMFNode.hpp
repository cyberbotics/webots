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

#ifndef WB_MF_NODE_HPP
#define WB_MF_NODE_HPP

//
// Description: field value that contains a multiple WbNode
//

#include "WbMultipleValue.hpp"

#include <QtCore/QVector>

#include <cassert>

class WbNode;

class WbMFNode : public WbMultipleValue {
  Q_OBJECT

public:
  typedef WbNode *WbNodePtr;
  typedef WbMFIterator<WbMFNode, WbNode *> Iterator;

  WbMFNode(WbTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  WbMFNode(const WbMFNode &other);
  virtual ~WbMFNode();
  WbValue *clone() const override { return new WbMFNode(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  int size() const override { return mVector.size(); }
  void clear() override;
  void insertDefaultItem(int index) override;
  WbVariant defaultNewVariant() const override { return WbVariant((WbNode *)NULL); }
  void removeItem(int index) override;  // remove and delete the node instance
  bool removeNode(WbNode *node);        // remove without deleting the node instance
  void writeItem(WbWriter &writer, int index) const override;
  WbVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return WbVariant(mVector[index]);
  }
  WbFieldType type() const override { return WB_MF_NODE; }
  const WbNodePtr &item(int index) const {
    assert(index >= 0 && index < size());
    return mVector[index];
  }
  void setItem(int index, WbNode *node);  // replace node at index and delete the previous node instance
  void addItem(WbNode *node);
  void insertItem(int index, WbNode *node);
  WbMFNode &operator=(const WbMFNode &other);
  bool operator==(const WbMFNode &other) const;
  int nodeIndex(const WbNode *node) const;
  void write(WbWriter &) const override;

protected:
  void readAndAddItem(WbTokenizer *tokenizer, const QString &worldPath) override;
  bool smallSeparator(int i) const override { return false; }

private:
  QVector<WbNode *> mVector;
  void defHasChanged() override;
};

#endif
