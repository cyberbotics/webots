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
 * Description:  Abstract class allowing to get Objects related with a specific robot
 */

#ifndef ROBOT_OBJECT_FACTORY_HPP
#define ROBOT_OBJECT_FACTORY_HPP

#include <QtCore/QString>

class RobotActuatorCommand;
class RobotSensorCondition;
class RobotFacade;
class RobotConditionWidget;
class RobotStateWidget;
class RobotViewWidget;
class Model;
class QWidget;

class RobotObjectFactory {
public:
  static RobotObjectFactory *instance();
  static void setInstance(RobotObjectFactory *instance);
  static void killInstance();

  virtual RobotFacade *createRobotFacade() const = 0;

  virtual RobotSensorCondition *createRobotSensorCondition() const = 0;
  virtual RobotActuatorCommand *createRobotActuatorCommand() const = 0;

  virtual RobotConditionWidget *createRobotConditionWidget(QWidget *parent = NULL) const = 0;
  virtual RobotStateWidget *createRobotStateWidget(QWidget *parent = NULL) const = 0;
  virtual RobotViewWidget *createRobotViewWidget(QWidget *parent = NULL) const = 0;

  const QString &name() const { return mRobotName; }

protected:
  explicit RobotObjectFactory(const QString &robotName);
  virtual ~RobotObjectFactory() {}

private:
  static bool cIsAlreadyAnInstance;
  static RobotObjectFactory *cInstance;

  QString mRobotName;
};

#endif
