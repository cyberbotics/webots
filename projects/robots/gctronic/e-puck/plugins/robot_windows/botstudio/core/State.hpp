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
 * Description:  Class defining the model of the state
 */

#ifndef STATE_HPP
#define STATE_HPP

#include "AutomatonObject.hpp"

#include <QtCore/QList>
#include <QtCore/QTextStream>

class Transition;
class RobotActuatorCommand;

QT_BEGIN_NAMESPACE
class QPointF;
QT_END_NAMESPACE

class State : public AutomatonObject {
  Q_OBJECT

public:
  explicit State(const QPointF &position);
  virtual ~State();

  bool isInitial() const { return mIsInitial; }
  bool isCurrent() const { return mIsCurrent; }
  RobotActuatorCommand *actuatorCommand() const { return mActuatorCommand; }

  void setInitial(bool isInitial);
  void setCurrent(bool isCurrent);

  void addTransition(Transition *t);
  void removeTransition(Transition *t);
  void removeTransitionAt(int i);
  void removeAllTransitions();
  QList<Transition *> transitions() const { return mTransition; }

  void fromString(const QString &string);
  void fromStringVersion3(const QString &string);  // backward compatibility code
  QString toString() const;

signals:
  void initialStateChanged();
  void currentStateChanged();

private:
  QList<Transition *> mTransition;
  bool mIsInitial;
  bool mIsCurrent;
  RobotActuatorCommand *mActuatorCommand;
};

#endif
