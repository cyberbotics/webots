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

#ifndef WB_NORMAL_HPP
#define WB_NORMAL_HPP

#include "WbBaseNode.hpp"
#include "WbMFVector3.hpp"

class WbVector3;

class WbNormal : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbNormal(WbTokenizer *tokenizer = NULL);
  WbNormal(const WbNormal &other);
  explicit WbNormal(const WbNode &other);
  virtual ~WbNormal();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_NORMAL; }

  // field accessors
  const WbMFVector3 &vector() const { return *mVector; }
  const WbVector3 &vector(int index) const { return mVector->item(index); }
  int vectorSize() const { return mVector->size(); }
  void setVector(int index, const WbVector3 &vector) { mVector->setItem(index, vector); }

  QStringList fieldsToSynchronizeWithX3D() const override;

private:
  // user accessible fields
  WbMFVector3 *mVector;

  WbNormal &operator=(const WbNormal &);  // non copyable
  WbNode *clone() const override { return new WbNormal(*this); }
  void init();
};

#endif
