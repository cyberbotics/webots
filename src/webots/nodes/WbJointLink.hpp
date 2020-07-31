// Copyright 1996-2020 Cyberbotics Ltd.
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

#ifndef WB_JOINT_PARAMETERS_HPP
#define WB_JOINT_PARAMETERS_HPP

#include "WbBaseNode.hpp"
#include "WbSFDouble.hpp"
#include "WbSFVector3.hpp"

class WbJointLink : public WbBaseNode {
  Q_OBJECT

public:
  explicit WbJointLink(const QString &modelName, WbTokenizer *tokenizer = NULL);
  explicit WbJointLink(WbTokenizer *tokenizer = NULL);
  WbJointLink(const WbJointLink &other);
  explicit WbJointLink(const WbNode &other);
  virtual ~WbJointLink();

  int nodeType() const override { return WB_NODE_JOINT_LINK; }
  void preFinalize() override;
  void postFinalize() override;

signals:

private:
  WbJointLink &operator=(const WbJointLink &);  // non copyable
  WbNode *clone() const override { return new WbJointLink(*this); }
  void init();

  // fields

private slots:
};

#endif
