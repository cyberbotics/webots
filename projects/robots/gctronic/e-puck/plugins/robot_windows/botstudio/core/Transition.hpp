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
 * Description:  Class defining the model of a transition
 */

#ifndef TRANSITION_HPP
#define TRANSITION_HPP

#include <QtCore/QTextStream>
#include "AutomatonObject.hpp"

class Automaton;
class State;
class RobotSensorCondition;

QT_BEGIN_NAMESPACE
class QPointF;
QT_END_NAMESPACE

class Transition : public AutomatonObject {
public:
  Transition(const QPointF &position, State *startState, State *endState);
  virtual ~Transition();

  void setStartState(State *startState) { mStartState = startState; }
  void setEndState(State *endState) { mEndState = endState; }

  State *startState() const { return mStartState; }
  State *endState() const { return mEndState; }
  RobotSensorCondition *sensorCondition() const { return mSensorCondition; }

  void fromString(const QString &string);
  void fromStringVersion3(const QString &string);  // backward compatibility code
  QString toString() const;

private:
  State *mStartState;
  State *mEndState;
  RobotSensorCondition *mSensorCondition;
};

#endif
