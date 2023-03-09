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

// Description:   Simple program to handle a cube with the IPR

#include "IPR.hpp"

#include <webots/DistanceSensor.hpp>
#include <webots/Robot.hpp>

using namespace webots;

#define OBJECT_NUMBER 4

double gGrabPosition[] = {3.00660, -1.35619, 1.19083, -3.24647, -2.94524, 0.727475, -0.727475};
double gDropPosition[] = {5.09282, 0.00000, 3.08698, -1.34990, -2.82252, 0.727475, -0.727475};

int main(int argc, char **argv) {
  IPR *ipr = new IPR();

  ipr->grabCube(gGrabPosition);
  ipr->dropCube(gDropPosition);
  ipr->moveToInitPosition();

  delete ipr;
  return 0;
}
