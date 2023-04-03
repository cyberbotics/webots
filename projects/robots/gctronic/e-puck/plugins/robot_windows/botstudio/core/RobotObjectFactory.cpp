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
 * Description:  Implementation of the RobotObjectFactory.hpp functions
 */

#include "RobotObjectFactory.hpp"

#include <QtCore/QtGlobal>

bool RobotObjectFactory::cIsAlreadyAnInstance = false;
RobotObjectFactory *RobotObjectFactory::cInstance = NULL;

RobotObjectFactory *RobotObjectFactory::instance() {
  return cInstance;
}

void RobotObjectFactory::setInstance(RobotObjectFactory *instance) {
  if (cIsAlreadyAnInstance)
    qFatal("Cannot create two instances of RobotObjectFactory");
  else {
    cInstance = instance;
    cIsAlreadyAnInstance = true;
  }
}

void RobotObjectFactory::killInstance() {
  if (cInstance) {
    delete cInstance;
    cInstance = NULL;
    cIsAlreadyAnInstance = false;
  }
}

RobotObjectFactory::RobotObjectFactory(const QString &robotName) : mRobotName(robotName) {
}
