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

#ifndef WB_SF_NODE_HPP
#define WB_SF_NODE_HPP

//
// Description: field value that contains a single WbNode
//

#include "WbSingleValue.hpp"

class WbNode;

class WbSFNode : public WbSingleValue {
  Q_OBJECT

public:
  WbSFNode(WbTokenizer *tokenizer, const QString &worldPath);
  WbSFNode(const WbSFNode &other);
  explicit WbSFNode(WbNode *node);
  virtual ~WbSFNode();
  void read(WbTokenizer *tokenizer, const QString &worldPath) override { readSFNode(tokenizer, worldPath); }
  void write(WbWriter &writer) const override;
  WbValue *clone() const override { return new WbSFNode(*this); }
  bool equals(const WbValue *other) const override;
  void copyFrom(const WbValue *other) override;
  WbVariant variantValue() const override { return WbVariant(mValue); }
  WbFieldType type() const override { return WB_SF_NODE; }
  WbNode *value() const { return mValue; }
  void setValue(WbNode *node);
  void removeValue() { setValue(NULL); }
  WbSFNode &operator=(const WbSFNode &other);
  bool operator==(const WbSFNode &other) const;

private:
  WbNode *mValue;
  void readSFNode(WbTokenizer *tokenizer, const QString &worldPath);
  void defHasChanged() override;
};

#endif
