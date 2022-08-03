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

#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>

#include "Thymio2AsebaHub.hpp"
#include "Thymio2Model.hpp"

#include "thymio2_definitions.h"

#include <common/consts.h>

#include <cstdio>  // sscanf
#include <iostream>

using namespace std;
using namespace webots;

static void welcomeMessage() {
  cout << "Thymio II Aseba server" << endl;
  cout << "- Connect Aseba Studio or VPL for Thymio II to this controller through TCP/IP." << endl;
  cout << "- Use the Thymio II robot window to access some special events (tap, clap, etc.)." << endl;
  cout << "- Change the server port using the Thymio2::controllerArgs field." << endl;
}

static void usage(const char *command) {
  cout << "Usage: " << command << " [port=33333]" << endl;
}

int main(int argc, char **argv) {
  welcomeMessage();

  Thymio2Model *thymio2 = new Thymio2Model;

  int port = ASEBA_DEFAULT_PORT;
  if (argc > 2) {
    cerr << "Invalid arguments" << endl;
    usage(argv[0]);
  } else if (argc == 2) {
    if (sscanf(argv[1], "port=%d", &port) != 1) {
      cerr << "Invalid arguments" << endl;
      usage(argv[0]);
      port = ASEBA_DEFAULT_PORT;
    }
  }

  Thymio2AsebaHub *hub = new Thymio2AsebaHub(port);

  bool events[EVENT_COUNT];

  while (thymio2->safeStep()) {
    hub->step();
    thymio2->sensorToHub(hub);
    thymio2->updateEvents(hub, events);
    hub->sendEvents(events);
    thymio2->hubToActuators(hub);
  }

  delete thymio2;

  return EXIT_SUCCESS;
}
