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

#ifndef WB_LINEAR_MOTOR_HPP
#define WB_LINEAR_MOTOR_HPP

#include "WbMotor.hpp"

class WbLinearMotor : public WbMotor {
  Q_OBJECT

public:
  explicit WbLinearMotor(WbTokenizer *tokenizer = NULL);
  WbLinearMotor(const WbLinearMotor &other);
  explicit WbLinearMotor(const WbNode &other);
  virtual ~WbLinearMotor();
  int nodeType() const override { return WB_NODE_LINEAR_MOTOR; }
  double force() const { return mMotorForceOrTorque; }
  double computeFeedback() const override;

protected:
  void turnOffMotor() override;

private:
  WbNode *clone() const override { return new WbLinearMotor(*this); }

  void init();
};

#endif
