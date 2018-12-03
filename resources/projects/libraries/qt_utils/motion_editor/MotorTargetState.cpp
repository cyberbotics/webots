#include "MotorTargetState.hpp"

#include "MotionGlobalSettings.hpp"

#include <devices/Motor.hpp>

#include <QtCore/qmath.h>

using namespace webotsQtUtils;

double MotorTargetState::defaultMinValue() {
  return -M_PI;
}

double MotorTargetState::defaultMaxValue() {
  return M_PI;
}

MotorTargetState::MotorTargetState(Motor *motor) :
  mMotor(motor),
  mSavedValue(0.0),
  mCurrentValue(0.0),
  mSavedDefined(false),
  mCurrentDefined(false),
  mIsValid(true),
  mIsModified(false) {
  updateRange();
}

MotorTargetState::MotorTargetState(const MotorTargetState &other) :
  mMotor(other.mMotor),
  mSavedValue(other.mSavedValue),
  mCurrentValue(other.mCurrentValue),
  mSavedDefined(other.mSavedDefined),
  mCurrentDefined(other.mCurrentDefined),
  mIsValid(other.mIsValid),
  mIsModified(other.mIsModified),
  mIsRangeUnlimited(other.mIsRangeUnlimited),
  mMinValue(other.mMinValue),
  mMaxValue(other.mMaxValue) {
}

MotorTargetState::~MotorTargetState() {
}

void MotorTargetState::updateRange() {
  if (mMotor) {
    mMinValue = mMotor->minPosition();
    mMaxValue = mMotor->maxPosition();
    if (mMinValue != mMaxValue) {
      mIsRangeUnlimited = false;
      return;
    }
  }

  // no limits
  mIsRangeUnlimited = true;
  mMinValue = defaultMinValue();
  mMaxValue = defaultMaxValue();
}

bool MotorTargetState::setValueFromParser(const QString &value) {
  mSavedValue = value.toDouble(&mSavedDefined);
  mCurrentDefined = mSavedDefined;
  mCurrentValue = mSavedValue;

  if (mSavedValue > mMaxValue || mSavedValue < mMinValue)
    mIsValid = false;

  emit updated();
  return mIsValid;
}

void MotorTargetState::setValue(double value) {
  mCurrentValue = value;
  if (mCurrentDefined)
    mMotor->setPosition(mCurrentValue);

  updateIsModified();
  updateIsValid();
  emit updated();
}

void MotorTargetState::reset() {
  mCurrentDefined = mSavedDefined;
  mCurrentValue = mSavedValue;

  updateIsModified();
  updateIsValid();
  emit updated();
}

void MotorTargetState::setDefined(bool defined) {
  mCurrentDefined = defined;
  updateIsModified();
  updateIsValid();
  emit updated();
}

QString MotorTargetState::toString() const {
  int p = MotionGlobalSettings::precision();

  if (mCurrentDefined)
    return QString("%1 %2").arg(precisionReducer(mCurrentValue), p + 3, 'g', p, QLatin1Char(' ')).arg(mMotor->name());
  else {
    QString spaces;
    for (int i = 0; i < (p + 3); i++)
      spaces.append(" ");
    return QString("%1 %2").arg(spaces).arg(mMotor->name());
  }
}

MotorTargetState::Status MotorTargetState::status() const {
  if (!mIsValid && (mCurrentDefined || !mIsModified))
    return INVALID;
  else if (mIsModified)
    return MODIFIED;
  else if (!mCurrentDefined)
    return DISABLED;
  else
    return NORMAL;
}

void MotorTargetState::updateAfterSaving() {
  mSavedValue = mCurrentValue;
  mSavedDefined = mCurrentDefined;
  updateIsModified();
  updateIsValid();
  emit updated();
}

void MotorTargetState::updateIsModified() {
  bool modified = (mCurrentDefined != mSavedDefined) || (mCurrentDefined && mSavedValue != mCurrentValue);
  if (modified != mIsModified) {
    mIsModified = modified;
    emit modifiedChanged(modified);
  }
}

void MotorTargetState::updateIsValid() {
  updateRange();
  bool valid = !mCurrentDefined || (mCurrentValue <= mMaxValue && mCurrentValue >= mMinValue);
  if (valid != mIsValid) {
    mIsValid = valid;
    emit validChanged();
  }
}

double MotorTargetState::precisionReducer(double value) {
  int p = MotionGlobalSettings::precision();
  int pow = qPow(10, p);
  double v = value;
  v *= pow;
  v = (int)v;
  v /= pow;
  return v;
}
