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
 * Description:  Implementation of the State.hpp functions
 */

#include "State.hpp"

#include "RobotActuatorCommand.hpp"
#include "RobotObjectFactory.hpp"
#include "Tokenizer.hpp"

#include <QtCore/QStringList>

State::State(const QPointF &position) : AutomatonObject(position), mIsInitial(false), mIsCurrent(false) {
  mActuatorCommand = RobotObjectFactory::instance()->createRobotActuatorCommand();
  setName(tr("State %1").arg(name()));
}

State::~State() {
  delete mActuatorCommand;
}

void State::addTransition(Transition *t) {
  mTransition.append(t);
}

void State::removeTransition(Transition *t) {
  mTransition.removeAll(t);
}

void State::removeTransitionAt(int i) {
  mTransition.removeAt(i);
}

void State::removeAllTransitions() {
  mTransition.clear();
}

void State::setInitial(bool isInitial) {
  mIsInitial = isInitial;
  emit initialStateChanged();
}

void State::setCurrent(bool isCurrent) {
  mIsCurrent = isCurrent;
  emit currentStateChanged();
}

void State::fromString(const QString &string) {
  Tokenizer tokenizer(string);
  QStringList tokens;

  for (int i = 0; i < 6; i++) {
    if (tokenizer.hasMoreToken())
      tokens.append(tokenizer.nextToken());
    else
      throw tr("Corrupted State (wrong number of parameters)");
  }

  QString sChar = tokens[0];
  int id = tokens[1].toInt();
  QString name = tokens[2].replace("\\n", "\n");
  int posX = tokens[3].toDouble();
  int posY = tokens[4].toDouble();
  bool isInitialBool = static_cast<bool>(tokens[5].toInt());

  if (sChar.size() != 1 || sChar[0] != 'S')
    throw tr("Corrupted State (first parameter isn't an S)");

  setPosition(QPointF(posX, posY));
  setName(name);
  setInitial(isInitialBool);
  setUniqueId(id);
  computeCurrentGlobalId(id);

  actuatorCommand()->fromString(tokenizer.remainingString());
}

QString State::toString() const {
  QString out;
  QString lName = name();
  lName.replace('\n', "\\n");
  lName.replace(';', ':');
  out += "S;";
  out += QString::number(uniqueId()) + ";";
  out += lName + ";";
  out += QString::number(position().x()) + ";";
  out += QString::number(position().y()) + ";";
  out += QString::number(isInitial()) + ";";
  out += mActuatorCommand->toString();
  return out;
}

// backward compatibility code
void State::fromStringVersion3(const QString &string) {
  Tokenizer tokenizer(string);
  QStringList tokens;

  for (int i = 0; i < 3; i++) {
    if (tokenizer.hasMoreToken())
      tokens.append(tokenizer.nextToken());
    else
      throw tr("Corrupted State (wrong number of parameters)");
  }

  QString name = tokens[0];
  int posX = tokens[1].toDouble() - 250;
  int posY = tokens[2].toDouble() - 200;
  setPosition(QPointF(posX, posY));
  setName(name);

  if (uniqueId() == 0)
    setInitial(true);

  actuatorCommand()->fromStringVersion3(tokenizer.remainingString());
}
