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

//
//  WbballJointParameters.hpp
//

#ifndef WB_BALL_JOINT_PARAMETERS_HPP
#define WB_BALL_JOINT_PARAMETERS_HPP

// Alias class for instantiation WbBallJoint's anchor parameter

#include "WbJointParameters.hpp"
#include "WbSFDouble.hpp"

class WbBallJointParameters : public WbJointParameters {
  Q_OBJECT

public:
  virtual ~WbBallJointParameters();
  WbBallJointParameters(const QString &modelName, WbTokenizer *tokenizer);
  explicit WbBallJointParameters(WbTokenizer *tokenizer = NULL);
  WbBallJointParameters(const WbBallJointParameters &other);
  explicit WbBallJointParameters(const WbNode &other);

  int nodeType() const override { return WB_NODE_BALL_JOINT_PARAMETERS; }
  void preFinalize() override;
  void postFinalize() override;

  virtual const WbVector3 &anchor() const { return mAnchor->value(); }

signals:
  void anchorChanged();

private:
  WbBallJointParameters &operator=(const WbBallJointParameters &);  // non copyable
  WbNode *clone() const override { return new WbBallJointParameters(*this); }
  void init();

  // fields
  WbSFVector3 *mAnchor;
};

#endif
