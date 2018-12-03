#include "entry_points.hpp"

#include <core/MainApplication.hpp>
#include <gui/GenericWindow.hpp>

#include <iostream>

using namespace webotsQtUtils;
using namespace std;

static MainApplication *gApplication = NULL;
static GenericWindow *gGenericWindow = NULL;

bool wbw_init() {
  cerr << "Warning: this robot window is deprecated. Please use the HTML5-based generic robot window instead by resetting the "
          "Robot.window field."
       << endl;

  gApplication = new MainApplication;
  if (gApplication->isInitialized())
    gGenericWindow = new GenericWindow;
  return gApplication->isInitialized();
}

void wbw_cleanup() {
  if (gGenericWindow) {
    delete gGenericWindow;
    gGenericWindow = NULL;
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
  if (gGenericWindow && gGenericWindow->isVisible())
    gGenericWindow->readSensors();
}

void wbw_write_actuators() {
  if (gGenericWindow && gGenericWindow->isVisible())
    gGenericWindow->writeActuators();
}

void wbw_show() {
  if (gGenericWindow)
    gGenericWindow->showWindow();
}
