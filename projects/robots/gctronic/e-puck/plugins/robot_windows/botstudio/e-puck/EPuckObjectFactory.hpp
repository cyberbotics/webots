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
 * Description:  Class allowing to get Objects related with a specific robot
 */

#ifndef EPUCK_OBJECT_FACTORY_HPP
#define EPUCK_OBJECT_FACTORY_HPP

#include <core/RobotObjectFactory.hpp>

class EPuckObjectFactory : public RobotObjectFactory {
public:
  EPuckObjectFactory();
  virtual ~EPuckObjectFactory();

  virtual RobotFacade *createRobotFacade() const;
  virtual RobotSensorCondition *createRobotSensorCondition() const;
  virtual RobotConditionWidget *createRobotConditionWidget(QWidget *parent = NULL) const;
  virtual RobotStateWidget *createRobotStateWidget(QWidget *parent = NULL) const;
  virtual RobotViewWidget *createRobotViewWidget(QWidget *parent = NULL) const;
  virtual RobotActuatorCommand *createRobotActuatorCommand() const;
};

#endif
