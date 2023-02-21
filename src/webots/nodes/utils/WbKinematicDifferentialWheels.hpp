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

#ifndef WB_KINEMATIC_DIFFERENTIAL_WHEELS_HPP
#define WB_KINEMATIC_DIFFERENTIAL_WHEELS_HPP

#include "WbVector2.hpp"

class WbBaseNode;
class WbCylinder;
class WbHingeJoint;
class WbRobot;

class WbKinematicDifferentialWheels {
public:
  virtual ~WbKinematicDifferentialWheels() {}
  static WbKinematicDifferentialWheels *createKinematicDifferentialWheelsIfNeeded(WbRobot *robot);

  // kinematic motion model
  void applyKinematicMotion(double ms);

  // kinematic collision model
  void applyKinematicDisplacement();
  void addKinematicDisplacement(WbVector2 displacement) {
    mKinematicDisplacement += displacement;
    mKinematicDisplacementNumber++;
  }

private:
  WbKinematicDifferentialWheels(WbRobot *robot, double wheelsRadius, double axleLength, WbHingeJoint *leftJoint,
                                WbHingeJoint *rightJoint);
  static WbCylinder *getRecursivelyBigestCylinder(WbBaseNode *node);
  // kinematic displacement (kinematic collision model)
  WbVector2 mKinematicDisplacement;
  int mKinematicDisplacementNumber;
  double mWheelsRadius;
  double mAxleLength;
  WbHingeJoint *mWheelJoints[2];
  WbRobot *mRobot;
};

#endif  // WB_KINEMATIC_DIFFERENTIAL_WHEELS_HPP
