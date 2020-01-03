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
//                Control the IPR2 to take the cube passed by the
//                other robot and throw it in the box.

#include "IPRCollaboration.hpp"

#define OBJECT_NUMBER 3

// IPR2 poses
const double gWaitPosition[] = {3.004520, -0.03308, 2.39263, -2.27029, 0.00000, 0.66207, -0.66207};
const double gGrabPosition[] = {3.004520, -0.03307, 3.47000, -2.27029, 0.00000, 0.66207, -0.66207};
const double gThrowPosition[] = {2.513930, -0.00000, 3.39598, -2.27029, -1.33815, 0.00000, -0.00000};

class IPR2Collaboration : public IPRCollaboration {
public:
  void throwCube() {
    moveToPosition(gThrowPosition, false);

    // rotate base
    setMotorPosition(BASE_MOTOR, 5.95391);

    // check distance from target
    while (true) {
      if (motorPosition(BASE_MOTOR) > 5.30)
        break;

      step(basicTimeStep());
    }

    // open gripper at the right moment
    setMotorPosition(RIGHT_GRIPPER_MOTOR, 0.72);
    setMotorPosition(LEFT_GRIPPER_MOTOR, -0.72);

    // raise arm so that it can throw better
    setMotorPosition(UPPER_ARM_MOTOR, -0.0330743);

    // wait until movement completed
    while (!positionReached(BASE_MOTOR, 5.95391))
      step(basicTimeStep());
    while (!positionReached(UPPER_ARM_MOTOR, -0.0330743))
      step(basicTimeStep());
    while (!positionReached(RIGHT_GRIPPER_MOTOR, 0.72))
      step(basicTimeStep());
    while (!positionReached(LEFT_GRIPPER_MOTOR, -0.72))
      step(basicTimeStep());
  };

  void takeCube() {
    waitForSignal(GIVE_CUBE);

    openGripper();

    moveToPosition(gGrabPosition);

    closeGripper();

    emitSignal(LEAVE_CUBE);

    waitForSignal(THROW_CUBE);
  };
};

int main(int argc, char **argv) {
  IPR2Collaboration *ipr = new IPR2Collaboration();

  for (int i = 0; i < OBJECT_NUMBER; ++i) {
    ipr->moveToPosition(gWaitPosition);
    ipr->takeCube();
    if (i < (OBJECT_NUMBER - 1))
      ipr->emitSignal(IPRCollaboration::GRAB_CUBE);
    ipr->throwCube();
  }

  ipr->moveToInitPosition();

  delete ipr;
  return 0;
}
