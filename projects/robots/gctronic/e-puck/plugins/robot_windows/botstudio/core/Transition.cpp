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
 * Description:  Implementation of the Transition.hpp functions
 */

#include "Transition.hpp"
#include "Automaton.hpp"
#include "Model.hpp"
#include "RobotObjectFactory.hpp"
#include "RobotSensorCondition.hpp"
#include "State.hpp"
#include "Tokenizer.hpp"

Transition::Transition(const QPointF &position, State *startState, State *endState) :
  AutomatonObject(position),
  mStartState(startState),
  mEndState(endState) {
  mSensorCondition = RobotObjectFactory::instance()->createRobotSensorCondition();
  setName(tr("Transition %1").arg(name()));
}

Transition::~Transition() {
  delete mSensorCondition;
}

void Transition::fromString(const QString &string) {
  Tokenizer tokenizer(string);
  QStringList tokens;

  for (int i = 0; i < 7; i++) {
    if (tokenizer.hasMoreToken())
      tokens.append(tokenizer.nextToken());
    else
      throw tr("Corrupted Transition (wrong number of parameters)");
  }

  QString sChar = tokens[0];
  int id = tokens[1].toInt();
  QString name = tokens[2].replace("\\n", "\n");
  int posX = tokens[3].toDouble();
  int posY = tokens[4].toDouble();
  int startId = tokens[5].toInt();
  int endId = tokens[6].toInt();

  State *startStateObject = Model::instance()->automaton()->findStateById(startId);
  State *endStateObject = Model::instance()->automaton()->findStateById(endId);

  if (sChar.size() != 1 || sChar[0] != 'T')
    throw tr("Corrupted Transition (first parameter isn't a T)");
  if (startStateObject == NULL || endStateObject == NULL)
    throw tr("Corrupted transition (bad reference to state)");

  setPosition(QPointF(posX, posY));
  setName(name);
  setStartState(startStateObject);
  setEndState(endStateObject);
  setUniqueId(id);
  computeCurrentGlobalId(id);

  sensorCondition()->fromString(tokenizer.remainingString());
}

QString Transition::toString() const {
  QString out;
  QString lName = name();
  lName.replace('\n', "\\n");
  lName.replace(';', ':');
  out += "T;";
  out += QString::number(uniqueId()) + ";";
  out += lName + ";";
  out += QString::number(position().x()) + ";";
  out += QString::number(position().y()) + ";";
  out += QString::number(startState()->uniqueId()) + ";";
  out += QString::number(endState()->uniqueId()) + ";";
  out += mSensorCondition->toString();
  return out;
}

// backward compatibility code
void Transition::fromStringVersion3(const QString &string) {
  Tokenizer tokenizer(string);
  QStringList tokens;

  for (int i = 0; i < 3; i++) {
    if (tokenizer.hasMoreToken())
      tokens.append(tokenizer.nextToken());
    else
      throw tr("Corrupted Transition (wrong number of parameters)");
  }

  QString name = tokens[0];
  int startId = tokens[1].toInt();
  int endId = tokens[2].toInt();

  State *startStateObject = Model::instance()->automaton()->findStateById(startId);
  State *endStateObject = Model::instance()->automaton()->findStateById(endId);

  if (startStateObject == NULL || endStateObject == NULL)
    throw tr("Corrupted transition (bad reference to state)");

  setPosition((startStateObject->position() + endStateObject->position()) / 2);
  setName(name);
  setStartState(startStateObject);
  setEndState(endStateObject);

  sensorCondition()->fromStringVersion3(tokenizer.remainingString());
}
