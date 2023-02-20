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
//  WbSliderJoint.hpp
//

// Implemented node class representing a slider joint (1 DOF, translation along a choosen axis)

#ifndef WB_SLIDER_JOINT_HPP
#define WB_SLIDER_JOINT_HPP

#include "WbJoint.hpp"

class WbLinearMotor;

class WbSliderJoint : public WbJoint {
  Q_OBJECT

public:
  explicit WbSliderJoint(WbTokenizer *tokenizer = NULL);
  WbSliderJoint(const WbSliderJoint &other);
  explicit WbSliderJoint(const WbNode &other);
  virtual ~WbSliderJoint();

  int nodeType() const override { return WB_NODE_SLIDER_JOINT; }
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  void computeEndPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const override;

  // return the axis of the joint with coordinates relative to the parent Solid; defaults to unit z-axis
  WbVector3 axis() const override;
  void updateEndPointZeroTranslationAndRotation() override;

public slots:
  bool setJoint() override;
  void updatePosition() override;

protected:
  WbLinearMotor *linearMotor() const;
  void updatePosition(double position) override;
  WbVector3 anchor() const override;
  void applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) override;

  void writeExport(WbWriter &writer) const override;

protected slots:
  void updateParameters() override;
  void updateMinAndMaxStop(double min, double max) override;

private:
  void applyToOdeAxis() override;
  void applyToOdeMinAndMaxStop() override;
};

#endif
