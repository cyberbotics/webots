// Copyright 1996-2022 Cyberbotics Ltd.
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

/*
 * Description:   This controller should be used to control a car using a real racing wheel
 *                It has been tested with the 'Logitech G27 and G29 Racing Wheel'. However,
 *                it should work with any joystick/racing wheel (if an appropriate custom
 *                configuration file is assed in argument, or the default one is adapted).
 */

#include <webots/vehicle/Driver.hpp>
#include "JoystickInterface.hpp"

#include <iostream>

using namespace webots;

int main(int argc, char **argv) {
  Driver *driver = new Driver();
  JoystickInterface *joystickInterface;
  if (argc > 1)
    joystickInterface = new JoystickInterface(driver, argv[1]);
  else
    joystickInterface = new JoystickInterface(driver);

  // main loop
  while (driver->step() != -1) {
    if (!joystickInterface->step()) {
      std::cerr << "Please connect a joystick before starting the controller." << std::endl;
      break;
    }
  }

  delete joystickInterface;
  delete driver;
  return 0;
}
