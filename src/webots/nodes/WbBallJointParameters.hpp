// Copyright 1996-2019 Cyberbotics Ltd.
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

//
//  WbballJointParameters.hpp
//

#ifndef WB_BALL_JOINT_PARAMETERS_HPP
#define WB_BALL_JOINT_PARAMETERS_HPP

// Alias class for instantiation WbBallJoint's anchor parameter

#include "WbAnchorParameter.hpp"
#include "WbSFDouble.hpp"

class WbBallJointParameters : public WbAnchorParameter {
  Q_OBJECT

public:
  virtual ~WbBallJointParameters();
  WbBallJointParameters(const QString &modelName, WbTokenizer *tokenizer);
  explicit WbBallJointParameters(WbTokenizer *tokenizer = NULL);
  WbBallJointParameters(const WbBallJointParameters &other);
  explicit WbBallJointParameters(const WbNode &other);
  void preFinalize() override;
  void postFinalize() override;
  WbNode *clone() const override { return new WbBallJointParameters(*this); }
  int nodeType() const override { return WB_NODE_BALL_JOINT_PARAMETERS; }
  double springConstant() const { return mSpringConstant->value(); }
  double dampingConstant() const { return mDampingConstant->value(); }

signals:
  void springAndDampingConstantsChanged();

private:
  WbBallJointParameters &operator=(const WbBallJointParameters &);  // non copyable
  void init();
  WbSFDouble *mSpringConstant;
  WbSFDouble *mDampingConstant;

private slots:
  void updateSpringConstant();
  void updateDampingConstant();
};

#endif
