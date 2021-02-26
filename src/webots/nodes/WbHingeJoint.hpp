// Copyright 1996-2021 Cyberbotics Ltd.
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

// Implemented node class representing an hinge (1 DOF, rotation along a choosen axis)

#ifndef WB_HINGE_JOINT_HPP
#define WB_HINGE_JOINT_HPP

#include "WbBasicHingeJoint.hpp"

class WbHingeJoint : public WbBasicHingeJoint {
  Q_OBJECT

public:
  explicit WbHingeJoint(const QString &modelName, WbTokenizer *tokenizer = NULL);
  explicit WbHingeJoint(WbTokenizer *tokenizer = NULL);
  WbHingeJoint(const WbHingeJoint &other);
  explicit WbHingeJoint(const WbNode &other);
  virtual ~WbHingeJoint();

  int nodeType() const override { return WB_NODE_HINGE_JOINT; }
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;

  void preFinalize() override;
  void postFinalize() override;

public slots:

protected:
protected slots:

private slots:

private:
  double mGearMass;
  WbSFNode *mStartPoint;

  void init();
};

#endif
