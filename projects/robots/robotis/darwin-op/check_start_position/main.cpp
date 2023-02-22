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

#include <cmath>
#include <cstdlib>
#include <iostream>

#include "LinuxDARwIn.h"

using namespace Robot;
using namespace std;

int main() {
  LinuxCM730 linux_cm730("/dev/ttyUSB0");
  CM730 cm730(&linux_cm730);
  if (cm730.Connect() == false) {
    cerr << "Fail to connect CM-730!" << endl;
    return EXIT_FAILURE;
  }

  int value = 0;
  int thresholdPos = 230;  // (20/360)*4096 => 20 degree
  int thresholdAcc = 32;   // 128/4 => 1/4 G
  const int position[20] = {1500, 2517, 1834, 2283, 2380, 1710, 2043, 2033, 2057, 2043,
                            1277, 2797, 3513, 571,  2843, 1240, 2077, 2037, 2050, 2173};  // Start position of each motor

  if (cm730.ReadByte(JointData::ID_R_SHOULDER_PITCH, MX28::P_VERSION, &value, 0) != CM730::SUCCESS) {
    cerr << "Cannot get the MX28 firmware version" << endl;
    return EXIT_FAILURE;
  }

  // Check legs motors positions
  for (int c = 6; c < 18; c++) {
    cm730.ReadWord((c + 1), MX28::P_PRESENT_POSITION_L, &value, 0);
    if (fabs(value - position[c]) > thresholdPos) {
      cerr << "The legs position are not secure" << endl;
      return EXIT_SUCCESS;
    }
  }

  // Check accelerometer => orientation
  bool accelerometerSuccess = true;
  cm730.ReadWord(CM730::ID_CM, CM730::P_ACCEL_X_L, &value, 0);
  if (fabs(value - 512) > thresholdAcc)
    accelerometerSuccess = false;
  cm730.ReadWord(CM730::ID_CM, CM730::P_ACCEL_Y_L, &value, 0);
  if (fabs(value - 512) > thresholdAcc)
    accelerometerSuccess = false;
  cm730.ReadWord(CM730::ID_CM, CM730::P_ACCEL_Z_L, &value, 0);
  if (fabs(value - 640) > thresholdAcc)
    accelerometerSuccess = false;

  if (!accelerometerSuccess) {
    cerr << "The robot orientation is not secure" << endl;
    return EXIT_SUCCESS;
  }

  cout << "Success" << endl;
  return EXIT_SUCCESS;
}
