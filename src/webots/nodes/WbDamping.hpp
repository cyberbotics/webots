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

#ifndef WB_DAMPING_HPP
#define WB_DAMPING_HPP

#include "WbBaseNode.hpp"
#include "WbSFDouble.hpp"

class WbDamping : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbDamping(WbTokenizer *tokenizer = NULL);
  WbDamping(const WbDamping &other);
  explicit WbDamping(const WbNode &other);
  virtual ~WbDamping();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_DAMPING; }
  void preFinalize() override;
  void postFinalize() override;

  // field accessors
  double linear() const { return mLinear->value(); }
  double angular() const { return mAngular->value(); }

signals:
  void changed();

private:
  // user accessible fields
  WbSFDouble *mLinear;
  WbSFDouble *mAngular;

  WbDamping &operator=(const WbDamping &);  // non copyable
  WbNode *clone() const override { return new WbDamping(*this); }
  void init();

private slots:
  void updateLinear();
  void updateAngular();
};

#endif
