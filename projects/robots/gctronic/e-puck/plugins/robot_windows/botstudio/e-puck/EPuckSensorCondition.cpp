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
 * Description:  Implementation of the EPuckSensorCondition.hpp functions
 */

#include "EPuckSensorCondition.hpp"
#include <core/Model.hpp>
#include <core/Tokenizer.hpp>
#include "EPuckFacade.hpp"

#include <cmath>

static bool gGroundSensorUseNotified = false;

class EPuckCondition {
public:
  EPuckCondition() : _value(0), _inverted(false) {}
  virtual ~EPuckCondition() {}

  int value() const { return _value; }
  bool isInverted() const { return _inverted; }

  void setValue(int value) { _value = value; }
  void setInverted(bool inverted) { _inverted = inverted; }

private:
  int _value;
  bool _inverted;
};

EPuckSensorCondition::EPuckSensorCondition() : RobotSensorCondition(), mStartTimer(0.0) {
  mEPuckFacade = static_cast<EPuckFacade *>(Model::instance()->robotFacade());

  for (int i = 0; i < 8; i++)
    mDistanceSensorConditions[i] = new EPuckCondition;
  for (int i = 0; i < 3; i++)
    mGroundSensorConditions[i] = new EPuckCondition;
  mTimerCondition = new EPuckCondition;
  mCameraCondition = new EPuckCondition;
}

EPuckSensorCondition::~EPuckSensorCondition() {
  for (int i = 0; i < EPuckFacade::NUMBER_OF_DISTANCE_SENSORS; i++)
    delete mDistanceSensorConditions[i];
  for (int i = 0; i < 3; i++)
    delete mGroundSensorConditions[i];
  delete mTimerCondition;
  delete mCameraCondition;
}

void EPuckSensorCondition::setDistanceSensorValue(int i, int v) {
  mDistanceSensorConditions[i]->setValue(v);
}

void EPuckSensorCondition::setDistanceSensorInverted(int i, bool v) {
  mDistanceSensorConditions[i]->setInverted(v);
}

int EPuckSensorCondition::distanceSensorValue(int i) const {
  return mDistanceSensorConditions[i]->value();
}

bool EPuckSensorCondition::distanceSensorInverted(int i) const {
  return mDistanceSensorConditions[i]->isInverted();
}

void EPuckSensorCondition::setGroundSensorValue(int i, int v) {
  mGroundSensorConditions[i]->setValue(v);
}

void EPuckSensorCondition::setGroundSensorInverted(int i, bool v) {
  mGroundSensorConditions[i]->setInverted(v);
}

int EPuckSensorCondition::groundSensorValue(int i) const {
  return mGroundSensorConditions[i]->value();
}

bool EPuckSensorCondition::groundSensorInverted(int i) const {
  return mGroundSensorConditions[i]->isInverted();
}

void EPuckSensorCondition::setTimerValue(int v) {
  mTimerCondition->setValue(v);
}

int EPuckSensorCondition::timerValue() const {
  return mTimerCondition->value();
}

void EPuckSensorCondition::setCameraValue(int v) {
  mCameraCondition->setValue(v);
}

void EPuckSensorCondition::setCameraInverted(bool v) {
  mCameraCondition->setInverted(v);
}

int EPuckSensorCondition::cameraValue() const {
  return mCameraCondition->value();
}

bool EPuckSensorCondition::cameraInverted() const {
  return mCameraCondition->isInverted();
}

bool EPuckSensorCondition::isFired() const {
  int threashold, value;
  bool inverted;

  for (int i = 0; i < EPuckFacade::NUMBER_OF_DISTANCE_SENSORS; i++) {
    threashold = mDistanceSensorConditions[i]->value();
    inverted = mDistanceSensorConditions[i]->isInverted();
    value = mEPuckFacade->distanceSensorValue(i);
    if (threashold) {
      if (inverted && value > threashold)
        return false;
      else if (!inverted && value < threashold)
        return false;
    }
  }

  for (int i = 0; i < EPuckFacade::NUMBER_OF_GROUND_SENSORS; i++) {
    threashold = mGroundSensorConditions[i]->value();
    inverted = mGroundSensorConditions[i]->isInverted();
    value = mEPuckFacade->groundSensorValue(i);
    if (threashold) {
      if (inverted && value > threashold)
        return false;
      else if (!inverted && value < threashold)
        return false;
    }
  }

  threashold = mCameraCondition->value();
  inverted = mCameraCondition->isInverted();
  value = mEPuckFacade->cameraValue();

  if (threashold) {
    if (!value)
      return false;

    if (inverted && value > threashold)
      return false;
    else if (!inverted && value < threashold)
      return false;
  }

  threashold = mTimerCondition->value();
  double timer = mEPuckFacade->timer() - mStartTimer;
  if (threashold && timer < 0.1 * threashold)
    return false;

  return true;
}

void EPuckSensorCondition::reset() {
  mStartTimer = mEPuckFacade->timer();
}

void EPuckSensorCondition::resetSensorsUseNotification() {
  gGroundSensorUseNotified = false;
}

void EPuckSensorCondition::fromString(const QString &string) {
  Tokenizer tokenizer(string);
  QStringList tokens;

  int nTokens = 2 * (EPuckFacade::NUMBER_OF_DISTANCE_SENSORS + EPuckFacade::NUMBER_OF_GROUND_SENSORS + 2);
  for (int i = 0; i < nTokens; i++) {
    if (tokenizer.hasMoreToken())
      tokens.append(tokenizer.nextToken());
    else
      throw QObject::tr("Corrupted EPuckActuatorCommand (wrong number of parameters)");
  }

  int k = 0;
  for (int i = 0; i < EPuckFacade::NUMBER_OF_DISTANCE_SENSORS; i++) {
    mDistanceSensorConditions[i]->setValue(tokens[k++].toInt());
    mDistanceSensorConditions[i]->setInverted(tokens[k++].toInt());
  }
  for (int i = 0; i < EPuckFacade::NUMBER_OF_GROUND_SENSORS; i++) {
    int value = tokens[k++].toInt();
    int inverted = tokens[k++].toInt();
    mGroundSensorConditions[i]->setValue(value);
    mGroundSensorConditions[i]->setInverted(inverted);

    if (value != 0 || inverted != 0) {
      if (!gGroundSensorUseNotified)
        gGroundSensorUseNotified = mEPuckFacade->notifyUseOfGroundSensor(i);
    }
  }
  mTimerCondition->setValue(tokens[k++].toInt());
  mTimerCondition->setInverted(tokens[k++].toInt());
  mCameraCondition->setValue(tokens[k++].toInt());
  mCameraCondition->setInverted(tokens[k++].toInt());
}

QString EPuckSensorCondition::toString() const {
  QString out;

  for (int i = 0; i < EPuckFacade::NUMBER_OF_DISTANCE_SENSORS; i++) {
    out += QString::number(mDistanceSensorConditions[i]->value()) + ";";
    out += QString::number(mDistanceSensorConditions[i]->isInverted()) + ";";
  }
  for (int i = 0; i < EPuckFacade::NUMBER_OF_GROUND_SENSORS; i++) {
    out += QString::number(mGroundSensorConditions[i]->value()) + ";";
    out += QString::number(mGroundSensorConditions[i]->isInverted()) + ";";
  }
  out += QString::number(mTimerCondition->value()) + ";";
  out += QString::number(mTimerCondition->isInverted()) + ";";
  out += QString::number(mCameraCondition->value()) + ";";
  out += QString::number(mCameraCondition->isInverted()) + ";";

  return out;
}

// backward compatibility code
void EPuckSensorCondition::fromStringVersion3(const QString &string) {
  Tokenizer tokenizer(string);

  // skip the two first tokens (obsolete)
  for (int i = 0; i < 2; i++) {
    if (tokenizer.hasMoreToken())
      tokenizer.nextToken();
    else
      throw QObject::tr("Corrupted EPuckActuatorCommand (wrong number of parameters)");
  }

  while (tokenizer.hasMoreToken()) {
    const QString &condition = tokenizer.nextToken();
    QStringList list = condition.split(':');
    if (list.size() != 2)
      throw QObject::tr("Corrupted EPuckActuatorCommand (wrong number of parameters)");

    int command = list[0].toInt();
    int value = list[1].toInt();

    if (command == 20) {  // TIMER
      mTimerCondition->setValue(abs(value) / 2);
      mTimerCondition->setInverted(value < 0);
    } else if (command == 21) {  // CAMERA
      mCameraCondition->setValue(10 * abs(value));
      mCameraCondition->setInverted(value < 0);
    } else if (command >= 4 && command <= 11) {  // distance sensors
      mDistanceSensorConditions[command - 4]->setValue(abs(value));
      mDistanceSensorConditions[command - 4]->setInverted(value < 0);
    } else if (command >= 12 && command <= 14) {  // ground sensors
      mGroundSensorConditions[command - 12]->setValue(abs(value));
      mGroundSensorConditions[command - 12]->setInverted(value < 0);
    }
  }
}
