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
//  WbRotationalMotor.hpp
//

#ifndef WB_ROTATIONAL_MOTOR_HPP
#define WB_ROTATIONAL_MOTOR_HPP

#include "WbMotor.hpp"

class WbRotationalMotor : public WbMotor {
  Q_OBJECT

public:
  virtual ~WbRotationalMotor();
  explicit WbRotationalMotor(WbTokenizer *tokenizer = NULL);
  WbRotationalMotor(const WbRotationalMotor &other);
  explicit WbRotationalMotor(const WbNode &other);
  double torque() const { return mMotorForceOrTorque; }
  int nodeType() const override { return WB_NODE_ROTATIONAL_MOTOR; }
  double computeFeedback() const override;

protected:
  void turnOffMotor() override;

private:
  WbNode *clone() const override { return new WbRotationalMotor(*this); }
  void init();
};

#endif
