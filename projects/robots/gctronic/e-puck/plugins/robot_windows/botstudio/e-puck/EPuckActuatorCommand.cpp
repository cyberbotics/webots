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
 * Description:  Implementation of the EPuckActuatorCommand.hpp functions
 */

#include "EPuckActuatorCommand.hpp"

#include <core/Model.hpp>
#include <core/Tokenizer.hpp>

EPuckActuatorCommand::EPuckActuatorCommand() : RobotActuatorCommand() {
  mEPuckFacade = static_cast<EPuckFacade *>(Model::instance()->robotFacade());

  for (int i = 0; i < EPuckFacade::NUMBER_OF_LEDS; i++)
    mLedValues[i] = EPuckFacade::NONE;
  for (int i = 0; i < 2; i++)
    mSpeeds[i] = 0;
}

void EPuckActuatorCommand::sendCommands() const {
  for (int i = 0; i < EPuckFacade::NUMBER_OF_LEDS; i++) {
    if (mLedValues[i] != EPuckFacade::NONE)
      mEPuckFacade->setLedValue(i, static_cast<bool>(mLedValues[i]));
  }
  mEPuckFacade->setSpeeds(mSpeeds[EPuckFacade::LEFT], mSpeeds[EPuckFacade::RIGHT]);
  mEPuckFacade->sendActuatorCommands();
}

void EPuckActuatorCommand::fromString(const QString &string) {
  Tokenizer tokenizer(string);
  QStringList tokens;

  for (int i = 0; i < (EPuckFacade::NUMBER_OF_LEDS + 2); i++) {
    if (tokenizer.hasMoreToken())
      tokens.append(tokenizer.nextToken());
    else
      throw QObject::tr("Corrupted EPuckActuatorCommand (wrong number of parameters)");
  }

  for (int i = 0; i < EPuckFacade::NUMBER_OF_LEDS; i++)
    mLedValues[i] = tokens[i].toInt();
  for (int i = 0; i < 2; i++)
    mSpeeds[i] = tokens[EPuckFacade::NUMBER_OF_LEDS + i].toInt();
}

QString EPuckActuatorCommand::toString() const {
  QString out;
  for (int i = 0; i < EPuckFacade::NUMBER_OF_LEDS; i++)
    out += QString::number(mLedValues[i]) + ";";
  for (int i = 0; i < 2; i++)
    out += QString::number(mSpeeds[i]) + ";";
  return out;
}

// backward compatibility code
void EPuckActuatorCommand::fromStringVersion3(const QString &string) {
  Tokenizer tokenizer(string);
  QStringList tokens;

  for (int i = 0; i < 8; i++) {
    if (tokenizer.hasMoreToken())
      tokens.append(tokenizer.nextToken());
    else
      throw QObject::tr("Corrupted EPuckActuatorCommand (wrong number of parameters)");
  }

  // get and set speeds
  mSpeeds[0] = tokens[3].toInt();
  mSpeeds[1] = tokens[4].toInt();

  // get and set leds
  int ledOn = tokens[5].toInt();
  int ledOff = tokens[6].toInt();
  for (int i = 0; i < EPuckFacade::NUMBER_OF_LEDS; i++) {
    if (ledOn & (1 << i))
      mLedValues[i] = EPuckFacade::ON;
    if (ledOff & (1 << i))
      mLedValues[i] = EPuckFacade::OFF;
  }
}
