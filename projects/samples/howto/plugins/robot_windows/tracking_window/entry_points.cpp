// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "entry_points.hpp"
#include "Tracker.hpp"

#include <core/MainApplication.hpp>

using namespace webotsQtUtils;

static MainApplication *gApplication = NULL;
static Tracker *gTracker = NULL;

bool wbw_init() {
  gApplication = new MainApplication;
  if (gApplication->isInitialized())
    gTracker = new Tracker;
  return gApplication->isInitialized();
}

void wbw_cleanup() {
  if (gTracker) {
    delete gTracker;
    gTracker = NULL;
  }
  if (gApplication) {
    delete gApplication;
    gApplication = NULL;
  }
}

void wbw_pre_update_gui() {
  if (gApplication && gApplication->isInitialized())
    gApplication->preUpdateGui();
}

void wbw_update_gui() {
  if (gApplication && gApplication->isInitialized())
    gApplication->updateGui();
}

void wbw_read_sensors() {
  if (gTracker && gTracker->isVisible())
    gTracker->readSensors();
}

void wbw_write_actuators() {
  if (gTracker && gTracker->isVisible())
    gTracker->writeActuators();
}

void wbw_show() {
  if (gTracker)
    gTracker->showWindow();
}
