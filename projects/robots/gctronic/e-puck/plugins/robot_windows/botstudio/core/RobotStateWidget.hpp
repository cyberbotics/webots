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
 * Description:  Class defining a widget allowing to view the selected state
 */

#ifndef ROBOT_STATE_WIDGET_HPP
#define ROBOT_STATE_WIDGET_HPP

#include <QtWidgets/QGraphicsView>

class Model;
class RobotActuatorCommand;

class RobotStateWidget : public QGraphicsView {
public:
  explicit RobotStateWidget(QWidget *parent = NULL);
  virtual ~RobotStateWidget() {}
  virtual void setActuatorCommand(RobotActuatorCommand *actuatorCommand) { mActuatorCommand = actuatorCommand; }
  RobotActuatorCommand *actuatorCommand() const { return mActuatorCommand; }

private:
  RobotActuatorCommand *mActuatorCommand;
};

#endif
