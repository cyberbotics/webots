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

// Description:   Sample showing how to use a Webots motion on the real robotis-op2

#ifndef MOTION_PLAYER_HPP
#define MOTION_PLAYER_HPP

#include <webots/Robot.hpp>

class MotionPlayer : public webots::Robot {
public:
  MotionPlayer();
  virtual ~MotionPlayer();
  void run();

private:
  int mTimeStep;

  void myStep();
  void wait(int ms);
};

#endif
