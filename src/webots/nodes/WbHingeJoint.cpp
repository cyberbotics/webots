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

#include "WbHingeJoint.hpp"

#include "WbBrake.hpp"
#include "WbHingeJointParameters.hpp"
#include "WbMathsUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeUtilities.hpp"
#include "WbRotationalMotor.hpp"
#include "WbSolid.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"

#include <cassert>

// Constructors

WbHingeJoint::WbHingeJoint(const QString &modelName, WbTokenizer *tokenizer) : WbBasicHingeJoint(modelName, tokenizer) {
  init();
}

WbHingeJoint::WbHingeJoint(WbTokenizer *tokenizer) : WbBasicHingeJoint("HingeJoint", tokenizer) {
  init();
}

WbHingeJoint::WbHingeJoint(const WbHingeJoint &other) : WbBasicHingeJoint(other) {
  init();
}

WbHingeJoint::WbHingeJoint(const WbNode &other) : WbBasicHingeJoint(other) {
  init();
}

WbHingeJoint::~WbHingeJoint() {
}

void WbHingeJoint::init() {
  mGearMass = findSFDouble("gearMass")->value();
  mStartPoint = findSFNode("startPoint");
}

void WbHingeJoint::preFinalize() {
  WbBasicHingeJoint::preFinalize();
}

void WbHingeJoint::postFinalize() {
  WbBasicHingeJoint::postFinalize();
}

void WbHingeJoint::prePhysicsStep(double ms) {
}

void WbHingeJoint::postPhysicsStep() {
}
