// Copyright 1996-2018 Cyberbotics Ltd.
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

  int nodeType() const override { return WB_NODE_BALL_JOINT_PARAMETERS; }
  void preFinalize() override;
  void postFinalize() override;

  double position() const { return mPosition->value(); }
  double maxStop() const { return mMaxStop->value(); }
  double minStop() const { return mMinStop->value(); }
  double springConstant() const { return mSpringConstant->value(); }
  double dampingConstant() const { return mDampingConstant->value(); }
  double staticFriction() const { return mStaticFriction->value(); }

  void setPosition(double p) { mPosition->setValue(p); }
  void setPositionFromOde(double p) { mPosition->setValueFromOde(p); }

signals:
  void positionChanged();
  void minAndMaxStopChanged(double min, double max);
  void springAndDampingConstantsChanged();

private:
  WbBallJointParameters &operator=(const WbBallJointParameters &);  // non copyable
  WbNode *clone() const override { return new WbBallJointParameters(*this); }
  void init();

  // fields
  WbSFDouble *mPosition;
  WbSFDouble *mMinStop;
  WbSFDouble *mMaxStop;
  WbSFDouble *mSpringConstant;
  WbSFDouble *mDampingConstant;
  WbSFDouble *mStaticFriction;

private slots:
  void updateMinAndMaxStop();
  void updateSpringConstant();
  void updateDampingConstant();
  void updateStaticFriction();
};

#endif
