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
 * Description:  Abstract class defining the way to get the robot information
 */

#ifndef ROBOT_FACADE_HPP
#define ROBOT_FACADE_HPP

#include <QtCore/QObject>

class RobotFacade : public QObject {
  Q_OBJECT

public:
  virtual ~RobotFacade() {}

  virtual void update() = 0;
  virtual void stop() = 0;
  virtual void sendActuatorCommands() = 0;

signals:
  void warning(const QString &text);
};

#endif
