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
 * Description:  Implementation of the Model.hpp functions
 */

#include "Model.hpp"

#include "Automaton.hpp"
#include "RobotFacade.hpp"
#include "RobotObjectFactory.hpp"

#include <QtCore/QRegularExpression>

Model *Model::cInstance = NULL;

Model *Model::instance() {
  if (!cInstance)
    cInstance = new Model;
  return cInstance;
}

void Model::killInstance() {
  delete cInstance;
  cInstance = NULL;
}

Model::Model() : QObject() {
  mAutomaton = new Automaton;
  mRobotFacade = RobotObjectFactory::instance()->createRobotFacade();
  mIsRunning = false;
}

Model::~Model() {
  delete mAutomaton;
  delete mRobotFacade;
}

void Model::updateValues() {
  mRobotFacade->update();
  if (mIsRunning)
    mAutomaton->run();
}

void Model::setRunning(bool isRunning) {
  mIsRunning = isRunning;
  mRobotFacade->stop();
  if (mIsRunning)
    mIsRunning = mAutomaton->prepareToRun();
  else
    mAutomaton->stop();
}

void Model::fromString(const QString &string) {
  if (string.size() < 1)
    throw tr("Empty file");

  int firstNewLineIndex = string.indexOf(QChar('\n'));
  if (firstNewLineIndex < 0)
    throw tr("Header not readable");

  QString header = string.left(firstNewLineIndex);
  QStringList headerList = header.split(QRegularExpression("[#. ]"), Qt::SkipEmptyParts);
  if (headerList.size() == 3 && headerList[0] == "botstudio") {
    int macro = headerList[1].toInt();
    int micro = headerList[2].toInt();
    if (macro == macroVersion() && micro == microVersion())
      mAutomaton->fromString(string);
    else if (macro == 3 && micro == 0)
      mAutomaton->fromStringVersion3(string);
    else
      throw tr("The BotStudio file is not readable, or its version is not supported by BotStudio");
  } else
    throw tr("Wrong header");
}

QString Model::toString() const {
  QString out;
  out += "#botstudio " + QString::number(macroVersion()) + "." + QString::number(microVersion()) + "\n";
  out += "#" + RobotObjectFactory::instance()->name() + "\n";
  out += "\n";
  out += mAutomaton->toString();
  return out;
}
