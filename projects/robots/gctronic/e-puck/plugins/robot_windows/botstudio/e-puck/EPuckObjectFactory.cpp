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
 * Description:  Implementation of the EPuckObjectFactory.hpp functions
 */

#include "EPuckObjectFactory.hpp"

#include "EPuckActuatorCommand.hpp"
#include "EPuckConditionWidget.hpp"
#include "EPuckFacade.hpp"
#include "EPuckSensorCondition.hpp"
#include "EPuckStateWidget.hpp"
#include "EPuckViewWidget.hpp"

EPuckObjectFactory::EPuckObjectFactory() : RobotObjectFactory("e-puck") {
}

EPuckObjectFactory::~EPuckObjectFactory() {
}

RobotFacade *EPuckObjectFactory::createRobotFacade() const {
  EPuckFacade *facade = new EPuckFacade;
  facade->sendActuatorCommands();
  return facade;
}

RobotSensorCondition *EPuckObjectFactory::createRobotSensorCondition() const {
  return new EPuckSensorCondition;
}

RobotActuatorCommand *EPuckObjectFactory::createRobotActuatorCommand() const {
  return new EPuckActuatorCommand;
}

RobotConditionWidget *EPuckObjectFactory::createRobotConditionWidget(QWidget *parent) const {
  return new EPuckConditionWidget(parent);
}

RobotStateWidget *EPuckObjectFactory::createRobotStateWidget(QWidget *parent) const {
  return new EPuckStateWidget(parent);
}

RobotViewWidget *EPuckObjectFactory::createRobotViewWidget(QWidget *parent) const {
  return new EPuckViewWidget(parent);
}
