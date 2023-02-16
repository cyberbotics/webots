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

/*
 * Description:  Implementation of the entry functions
 */

#include "entry_points.hpp"

#include <core/MainApplication.hpp>

#include <e-puck/EPuckObjectFactory.hpp>

#include "BotStudioPaths.hpp"
#include "BotStudioWindow.hpp"
#include "Model.hpp"

#include <webots/robot.h>

#include <QtWidgets/QMessageBox>

using namespace webotsQtUtils;

static MainApplication *gApplication = NULL;
static BotStudioWindow *gMainWindow = NULL;

bool wbw_init() {
  gApplication = new MainApplication;
  if (gApplication->isInitialized()) {
    // set main icon
    QApplication *app = static_cast<QApplication *>(QApplication::instance());
    app->setWindowIcon(QIcon(BotStudioPaths::getIconsPath() + "botstudio.png"));

    // create the Robot Factory
    // this has to be extended to set another robot
    EPuckObjectFactory *factory;
    QString model(wb_robot_get_model());
    model = model.simplified().replace("-", "");
    if (model.contains("epuck", Qt::CaseInsensitive))
      factory = new EPuckObjectFactory;
    else {
      QMessageBox::critical(NULL, "BotStudio",
                            QObject::tr("Robot not supported. Please check that your robot model is supported by BotStudio "
                                        "and that the Robot::model field is correctly setup"));
      return false;
    }
    RobotObjectFactory::setInstance(factory);

    // create the Gui
    gMainWindow = new BotStudioWindow;
  }
  return gApplication->isInitialized();
}

void wbw_cleanup() {
  if (gMainWindow) {
    delete gMainWindow;
    gMainWindow = NULL;
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
  if (gMainWindow && gMainWindow->isVisible()) {
    Model::instance()->updateValues();
    gMainWindow->updateValues();
  }
}

void wbw_write_actuators() {
  // TODO: possible improvement: send the actuator commands only there
}

void wbw_show() {
  if (gMainWindow)
    gMainWindow->showWindow();
}
