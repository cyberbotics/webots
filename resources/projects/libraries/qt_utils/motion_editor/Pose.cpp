#include "Pose.hpp"

#include "MotorTargetState.hpp"

#include <devices/Motor.hpp>

#include <QtCore/QStringList>

#include <cassert>

using namespace webotsQtUtils;

Pose::Pose() : mName("Unnamed"), mTime(0.0), mIsValid(true), mIsModified(false) {
}

Pose::Pose(const Pose &other) : mName(other.mName), mTime(other.mTime), mIsValid(other.mIsValid), mIsModified(false) {
  int size = other.mStates.size();
  mStates.reserve(size);
  foreach (const MotorTargetState *item, other.mStates) {
    MotorTargetState *state = new MotorTargetState(*item);
    mStates.append(state);
    connect(state, SIGNAL(updated()), this, SLOT(propagateStateUpdate()));
    connect(state, SIGNAL(validChanged()), this, SLOT(updateIsValid()));
    connect(state, SIGNAL(modifiedChanged(bool)), this, SLOT(updateIsModified(bool)));
  }
}

Pose::~Pose() {
  clear();
}

void Pose::setTimeFromParser(const QString &time) {
  QStringList timeTokens = time.split(':', Qt::SkipEmptyParts);
  if (timeTokens.size() != 3)
    throw tr("Error while parsing time: \"%1\"").arg(time);

  bool ok;
  int m = timeTokens[0].toInt(&ok);
  if (!ok)
    throw tr("Error while parsing time: \"%1\"").arg(time);
  int s = timeTokens[1].toInt(&ok);
  if (!ok)
    throw tr("Error while parsing time: \"%1\"").arg(time);
  int ms = timeTokens[2].toInt(&ok);
  if (!ok)
    throw tr("Error while parsing time: \"%1\"").arg(time);

  mTime = ms + 1000 * s + 60000 * m;

  emit updated();
}

void Pose::setNameFromParser(const QString &name) {
  mName = name;

  emit updated();
}

bool Pose::setMotorValueFromParser(Motor *motor, const QString &value) {
  MotorTargetState *state = createOrRetrieve(motor);
  bool valid = state->setValueFromParser(value);
  mIsValid &= valid;

  emit updated();
  return valid;
}

void Pose::setName(const QString &name) {
  mName = name;
  emit updated();
}

void Pose::setTime(int time) {
  mTime = time;
  emit updated();
}

void Pose::addNewMotor(Motor *motor) {
  assert(!findStateByMotorName(motor->name()));

  MotorTargetState *state = new MotorTargetState(motor);
  mStates << state;
  connect(state, SIGNAL(updated()), this, SLOT(propagateStateUpdate()));
  connect(state, SIGNAL(validChanged()), this, SLOT(updateIsValid()));
  connect(state, SIGNAL(modifiedChanged(bool)), this, SLOT(updateIsModified(bool)));

  emit updated();
}

MotorTargetState *Pose::createOrRetrieve(Motor *motor) {
  foreach (MotorTargetState *state, mStates) {
    if (motor->name() == state->motor()->name())
      return state;
  }
  MotorTargetState *state = new MotorTargetState(motor);
  mStates << state;
  connect(state, SIGNAL(updated()), this, SLOT(propagateStateUpdate()));
  connect(state, SIGNAL(validChanged()), this, SLOT(updateIsValid()));
  connect(state, SIGNAL(modifiedChanged(bool)), this, SLOT(updateIsModified(bool)));
  return state;
}

void Pose::clear() {
  foreach (MotorTargetState *state, mStates)
    delete state;
  mStates.clear();
}

MotorTargetState *Pose::findStateByMotorName(const QString &motorName) const {
  foreach (MotorTargetState *state, mStates)
    if (state->motor()->name() == motorName)
      return state;
  return NULL;
}

int Pose::computeIndexOfStateByMotorName(const QString &motorName) const {
  int index = 0;
  foreach (MotorTargetState *state, mStates) {
    if (state->motor()->name() == motorName)
      return index;
    index++;
  }
  return -1;
}

int Pose::milliSeconds() const {
  return (int)mTime % 1000;
}

int Pose::seconds() const {
  return (((int)mTime - milliSeconds()) / 1000) % 60;
}

int Pose::minutes() const {
  return ((int)mTime - 1000 * seconds() - milliSeconds()) / 60000;
}

QString Pose::toString() const {
  return QString("%1 %2").arg(toTimeString()).arg(mName);
}

QString Pose::toTimeString() const {
  return QString("%1:%2:%3")
    .arg(minutes(), 2, 10, QChar('0'))
    .arg(seconds(), 2, 10, QChar('0'))
    .arg(milliSeconds(), 3, 10, QChar('0'));
}

Pose::Status Pose::status() const {
  if (!mIsValid)
    return INVALID;
  else if (mIsModified)
    return MODIFIED;
  else
    return NORMAL;
}

void Pose::createAndAppendState(Motor *motor) {
  MotorTargetState *state = new MotorTargetState(motor);
  mStates << state;
  connect(state, SIGNAL(updated()), this, SLOT(propagateStateUpdate()));
  emit stateInserted(mStates.count());
}

void Pose::deleteStateAt(int index) {
  if (index >= 0 && index < mStates.count()) {
    MotorTargetState *state = mStates.takeAt(index);
    delete state;
    emit stateDeleted(index);
  }
}

void Pose::swapStateWithNext(int index) {
  if (index >= 0 && index < mStates.count() - 1) {
    MotorTargetState *state = mStates.takeAt(index);
    emit stateDeleted(index);
    mStates.insert(index + 1, state);
    emit stateInserted(index + 1);
  }
}

void Pose::propagateStateUpdate() {
  MotorTargetState *state = dynamic_cast<MotorTargetState *>(sender());
  if (state) {
    int index = computeStateToIndex(state);
    assert(index != -1);
    emit stateUpdated(index);
  }
}

int Pose::computeStateToIndex(MotorTargetState *s) const {
  for (int index = 0; index < mStates.count(); index++)
    if (s == mStates[index])
      return index;
  return -1;
}

void Pose::updateIsValid() {
  // recompute flag
  bool isValid = true;
  foreach (MotorTargetState *state, mStates) {
    if (!state->isValid()) {
      isValid = false;
      break;
    }
  }

  if (mIsValid != isValid) {
    mIsValid = isValid;
    emit updated(true);
  }
}

void Pose::updateIsModified(bool modified) {
  if (modified && !mIsModified) {
    mIsModified = modified;
    emit updated(true);

  } else if (mIsModified != modified) {
    // recompute flag
    modified = false;
    foreach (MotorTargetState *state, mStates) {
      if (state->isModified()) {
        modified = true;
        break;
      }
    }

    if (mIsModified != modified) {
      mIsModified = modified;
      emit updated(true);
    }
  }
}
