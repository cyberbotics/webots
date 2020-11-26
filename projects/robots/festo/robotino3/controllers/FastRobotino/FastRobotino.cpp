// Copyright 1996-2020 Cyberbotics Ltd.
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

#include <math.h>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

using namespace webots;
using namespace std;

Motor *motor[3];

// move forward vx m/s, left vy m/s, rotate left va radians/s (va=Angular
// Velocity)
void setSpeeds(double vx, double vy, double va) {
  vx /= 0.063; // wheel radius = 0.063
  vy /= 0.063;
  va *= 0.1826 / 0.063; // distance wheel center to robot center 0.1826
  motor[0]->setVelocity(vy + va);
  motor[1]->setVelocity(-0.5 * vy - sqrt(0.75) * vx + va);
  motor[2]->setVelocity(-0.5 * vy + sqrt(0.75) * vx + va);
}

int main(int argc, char *argv[]) {
  Robot *robot = new Robot();
  double timeStep = robot->getBasicTimeStep();
  Keyboard *keyboard = robot->getKeyboard();
  keyboard->enable(timeStep);
  for (int i = 3; i--;) {
    motor[i] = robot->getMotor("wheel" + to_string(i) + "_joint");
    motor[i]->setPosition(INFINITY);
    motor[i]->setVelocity(0);
  }
  double actualSpeed[3] = {0, 0, 0};
  // this acceleration was tested with WorldInfo.basicTimeStep=16, 32, 64
  // and velocity 8 for x or y or angular
  // extremly accuate, except angular velocity may differ from set speed by 10%
  double maxAcceleration[3] = {10, 6, 20};
  double targetSpeed[3] = {0, 0, 0};
  while (robot->step(timeStep) != -1) {
    double keySpeed[3] = {0, 0, 0};
    int key;
    while ((key = keyboard->getKey()) != -1) {
      if (key == 'W' || key == keyboard->UP)
        keySpeed[0] = 1;
      if (key == 'S' || key == keyboard->DOWN)
        keySpeed[0] = -1;
      if (key == 'A' || key == keyboard->LEFT)
        keySpeed[1] = 1;
      if (key == 'D' || key == keyboard->RIGHT)
        keySpeed[1] = -1;
      if (key == 'Q')
        keySpeed[2] = -1;
      if (key == 'E')
        keySpeed[2] = 1;
    }
    //    if(key==' ' || keySpeed[0] || keySpeed[1] || keySpeed[2])
    for (int i = 3; i--;)
      targetSpeed[i] = keySpeed[i] * 6;
    double maxSteps = 0;
    for (int i = 3; i--;) {
      double stepsNeeded = abs(targetSpeed[i] - actualSpeed[i]) /
                           (maxAcceleration[i] * (timeStep / 1000.0));
      if (stepsNeeded > maxSteps)
        maxSteps = stepsNeeded;
    }
    if (maxSteps < 1)
      maxSteps = 1;
    for (int i = 3; i--;)
      actualSpeed[i] += (targetSpeed[i] - actualSpeed[i]) / maxSteps;
    setSpeeds(actualSpeed[0], actualSpeed[1], actualSpeed[2]);
  }
}
