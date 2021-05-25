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

#include "entry_points.hpp"

#include <core/MainApplication.hpp>
#include "FireBird6Representation.hpp"
#include "Gui.hpp"

using namespace webotsQtUtils;

static MainApplication *gApplication = NULL;
static Gui *gGui = NULL;

bool wbw_init() {
  gApplication = new MainApplication;
  if (gApplication->isInitialized())
    gGui = new Gui;
  return gApplication->isInitialized();
}

void wbw_cleanup() {
  if (gGui) {
    delete gGui;
    gGui = NULL;
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
  if (gGui && gGui->isVisible()) {
    FireBird6Representation::instance()->update();
    gGui->updateValues();
  }
}

void wbw_write_actuators() {
  // nop: this gui is only a viewer
}

void wbw_show() {
  if (gGui)
    gGui->showWindow();
}
