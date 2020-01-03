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

// Description:   Example of collaborative work between two IPRs.
//                Communication between robots is achieved using
//                Emitter and Receiver devices.
//                Control the IPR1 to grab and pass the cube.

#include "IPRCollaboration.hpp"

#define OBJECT_NUMBER 3

// IPR1 poses
const double gGrabPositions[3][IPRCollaboration::MOTOR_NUMBER] = {
  {0.390582, -2.26583, 1.91850, -2.88388, -2.45437, 0.66207, -0.66207},
  {0.000010, -2.27120, 1.91850, -2.82253, -3.00660, 0.66207, -0.66207},
  {5.680000, -2.25000, 1.86000, -2.91500, -0.40000, 0.90000, -0.90000}};

const double gDropPosition[] = {3.016690, -0.86002, 0.77181, -1.96350, -1.22718, 0.66207, -0.66207};

class IPR1Collaboration : public IPRCollaboration {
public:
  void giveCube() {
    setMotorPosition(UPPER_ARM_MOTOR, -0.726919);
    simulationStep(5);
    moveToPosition(gDropPosition, false);

    emitSignal(GIVE_CUBE);

    waitForSignal(LEAVE_CUBE);

    openGripper(1.0);

    // rotate wrist
    setMotorPosition(ROTATIONAL_WRIST_MOTOR, -3.015);
    while (!positionReached(ROTATIONAL_WRIST_MOTOR, -3.015))
      step(basicTimeStep());

    // raise arm
    setMotorPosition(UPPER_ARM_MOTOR, 0.0);
    while (!positionReached(UPPER_ARM_MOTOR, 0.0))
      step(basicTimeStep());

    emitSignal(THROW_CUBE);
  };
};

int main(int argc, char **argv) {
  IPR1Collaboration *ipr = new IPR1Collaboration();
  /*while(true) {
    ipr->simulationStep();
  }*/
  for (int i = 0; i < OBJECT_NUMBER; ++i) {
    if (i > 0)
      ipr->waitForSignal(IPRCollaboration::GRAB_CUBE);

    ipr->grabCube(gGrabPositions[i]);
    ipr->giveCube();
  }

  ipr->moveToInitPosition();

  delete ipr;
  return 0;
}
