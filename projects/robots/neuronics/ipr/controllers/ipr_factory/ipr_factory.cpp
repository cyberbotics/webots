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

// Description:   Example of factory robots picking objects
//                from a conveyor belt.

#include "IPR.hpp"

#include <webots/DistanceSensor.hpp>
using namespace webots;

#define OBJECT_NUMBER 4

const double gGrabPosition[] = {4.52928, -1.52158, 1.15641, -3.10416, -1.45507, 0.66207, -0.66207};
const double gDropPositions[4][IPR::MOTOR_NUMBER] = {{1.87146, -1.72104, 1.49855, -3.30419, -2.67894, 0.66207, -0.66207},
                                                     {0.98690, -1.66282, 1.49855, -3.32408, -3.48962, 0.66207, -0.66207},
                                                     {1.35628, -1.61453, 1.20403, -3.13472, -1.63780, 0.66207, -0.66207},
                                                     {1.52564, -1.62196, 1.23645, -3.15043, -4.43836, 0.66207, -0.66207}};

class IPRFactory : public IPR {
public:
  IPRFactory() : IPR() {
    mBeltDistanceSensor = getDistanceSensor("dsBelt");
    if (mBeltDistanceSensor)
      mBeltDistanceSensor->enable(basicTimeStep());
  }

  virtual bool objectDetectedInGripper() const {
    double valueCenter = distanceSensorValue(4);
    double valueRight = distanceSensorValue(5);
    return (valueCenter > 100) && (valueRight > 50);
  }

  bool objectDetectedOnBelt() const {
    if (!mBeltDistanceSensor)
      return false;

    return mBeltDistanceSensor->getValue() > 100;
  }

private:
  DistanceSensor *mBeltDistanceSensor;
};

int main(int argc, char **argv) {
  IPRFactory *ipr = new IPRFactory();

  for (int i = 0; i < OBJECT_NUMBER; ++i) {
    while (!ipr->objectDetectedOnBelt())
      ipr->simulationStep();
    ipr->grabCube(gGrabPosition);
    ipr->dropCube(gDropPositions[i]);
  }

  ipr->moveToInitPosition();

  delete ipr;
  return 0;
}
