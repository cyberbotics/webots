#include "Motion.hpp"

#include "MotionGlobalSettings.hpp"
#include "MotionPlayer.hpp"
#include "MotorTargetState.hpp"
#include "Pose.hpp"

#include <core/StandardPaths.hpp>
#include <devices/Motor.hpp>

#include <webots/robot.h>

#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtCore/QTextStream>
#include <QtWidgets/QMessageBox>

#include <cassert>

using namespace webotsQtUtils;

Motion *Motion::cInstance = NULL;
Motion *Motion::instance() {
  return cInstance;
}

Motion::Motion(const MotionPlayer *player, const QString &filepath) :
  mNewPoseCounter(0),
  mPlayer(player),
  mIsValid(true),
  mIsPoseSelectionBlocked(false) {
  mFixedStep = wb_robot_get_basic_time_step();
  assert(!cInstance);
  setFilePath(filepath, true);
  load();
  cInstance = this;
}

Motion::~Motion() {
  clear();
  cInstance = NULL;
}

void Motion::setFilePath(const QString &filepath, bool checkExistence) {
  if (!checkExistence || (!filepath.isEmpty() && QFile::exists(filepath))) {
    mFilePath = filepath;
    mDefaultFilePath = false;
  } else {
    mFilePath = StandardPaths::getProjectPath() + "Untitled.motion";
    mDefaultFilePath = true;
  }

  mName = QFileInfo(mFilePath).baseName();

  emit updated();
}

void Motion::save() const {
  if (!validate()) {
    QMessageBox::critical(0, tr("Save motion file"), tr("The internal motion is corrupted and cannot be saved into a file"));
    return;
  }

  QFile file(mFilePath);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QMessageBox::critical(0, tr("Save motion file"), tr("Cannot open the target file in write mode"));
    return;
  }

  QTextStream out(&file);

  out << "#WEBOTS_MOTION,V1.0";

  if (mPoses.count() == 0)
    return;

  QList<Device *> motors;
  foreach (MotorTargetState *state, mPoses[0]->states()) {
    Device *motor = state->motor();
    motors << motor;
    out << "," << motor->name();
  }
  out << "\n";

  foreach (Pose *pose, mPoses) {
    out << pose->toTimeString() << "," << pose->name();

    foreach (Device *motor, motors) {
      MotorTargetState *state = pose->findStateByMotorName(motor->name());
      out << ",";
      if (state->isDefined())
        out << state->value();
      else
        out << "*";

      state->updateAfterSaving();
    }
    out << "\n";
  }
}

void Motion::applyPoseToRobot(Pose *pose) {
  if (pose && !mPlayer->isPlaying()) {
    foreach (MotorTargetState *state, pose->states()) {
      if (state->isDefined())
        state->motor()->setPosition(state->value());
    }
  }
}

bool Motion::estimateHasFixedStep() const {
  if (mPoses.count() <= 2)
    return true;

  int referenceDeltaTime = mPoses[0]->time() - mPoses[1]->time();

  for (int i = 0; i < mPoses.count() - 1; i++) {
    int currentDeltaTime = mPoses[i]->time() - mPoses[i + 1]->time();
    if (currentDeltaTime != referenceDeltaTime)
      return false;
  }

  return true;
}

void Motion::setFixedStep(int step) {
  mFixedStep = qMax(0, step);

  if (mFixedStep) {
    for (int i = 0; i < mPoses.count(); i++)
      mPoses[i]->setTime(step * i);
  }
}

int Motion::estimateFixedStepValue() const {
  if (mPoses.count() <= 2)
    return wb_robot_get_basic_time_step();

  int referenceDeltaTime = qAbs(mPoses[1]->time() - mPoses[0]->time());
  referenceDeltaTime = qMax(1, qMin(100, referenceDeltaTime));

  return referenceDeltaTime;
}

bool Motion::validate() const {
  if (mPoses.count() == 0)
    return true;

  QList<Device *> motors;
  foreach (MotorTargetState *state, mPoses[0]->states())
    motors << state->motor();

  foreach (Pose *pose, mPoses) {
    if (pose->states().count() != motors.count())
      return false;

    foreach (Device *motor, motors) {
      if (pose->findStateByMotorName(motor->name()) == NULL)
        return false;
    }
  }

  return true;
}

void Motion::load() {
  if (!QFile::exists(mFilePath))
    return;

  try {
    parse();
  } catch (QString &e) {
    QMessageBox::information(0, tr("Cannot load motion file"),
                             tr("Cannot load motion file \"%1\" due to this error: %2").arg(mFilePath).arg(e));
    clear();
  }
}

void Motion::parse() {
  if (!QFile::exists(mFilePath))
    return;

  QFile file(mFilePath);
  if (!file.open(QIODevice::ReadOnly))
    throw tr("Cannot read file: %1").arg(file.errorString());

  QTextStream in(&file);
  if (in.atEnd())
    throw tr("Empty file");

  // parse header
  const QString &firstLine = in.readLine();
  QStringList firstLineTokens = firstLine.split(',', Qt::SkipEmptyParts);

  if (firstLineTokens.size() < 2)
    throw tr("Invalid header: not enough tokens");

  if (firstLineTokens.takeFirst() != "#WEBOTS_MOTION")
    throw tr("Invalid header: first token is not #WEBOTS_MOTION");

  if (firstLineTokens.takeFirst() != "V1.0")
    throw tr("Invalid header: Invalid version");

  // get the list of motor devices used in the motion
  QStringList usedMotorNamesList(firstLineTokens);
  QList<Motor *> usedMotorList;
  foreach (const QString &motorName, usedMotorNamesList) {
    bool ok = false;
    foreach (Motor *motor, MotionGlobalSettings::availableMotorList()) {
      if (motor->name() == motorName) {
        ok = true;
        usedMotorList << motor;
      }
    }
    if (!ok)
      throw tr("The motor \"%1\" found in the motion file doesn't exist on this Robot").arg(motorName);
  }

  // point of no return accoriding to the error tolerance
  clear();

  // parse following lines
  int lineCounter = 0;
  int previousPoseTime = 0;
  while (!in.atEnd()) {
    lineCounter++;

    const QString &line = in.readLine();
    if (line.isEmpty())
      continue;  // support empty lines

    QStringList lineTokens = line.split(',', Qt::SkipEmptyParts);

    if (lineTokens.size() != usedMotorNamesList.size() + 2)
      throw tr("Cannot parse line: %1").arg(lineCounter);

    Pose *pose = new Pose;
    pose->setTimeFromParser(lineTokens.takeFirst());
    pose->setNameFromParser(lineTokens.takeFirst());
    int newPoseTime = pose->time();

    if (newPoseTime < previousPoseTime) {
      delete pose;
      throw tr("Pose at line %1 is corrupted. Poses should be sorted by time chronologically").arg(lineCounter);
    }
    previousPoseTime = newPoseTime;

    for (int i = 0; i < usedMotorList.size(); i++)
      mIsValid &= pose->setMotorValueFromParser(usedMotorList[i], lineTokens[i]);

    connect(pose, SIGNAL(updated(bool)), this, SLOT(propagatePoseUpdate()));
    connect(pose, SIGNAL(selected()), this, SLOT(propagateSelection()));

    mPoses << pose;
  }

  if (estimateHasFixedStep())
    mFixedStep = estimateFixedStepValue();
  else
    mFixedStep = 0;
}

void Motion::clear() {
  while (mPoses.count() > 0) {
    delete mPoses.takeFirst();
    emit poseDeleted(0);
  }
  mPoses.clear();
  mIsValid = true;
}

int Motion::computePoseToIndex(Pose *p) const {
  for (int index = 0; index < mPoses.count(); index++)
    if (p == mPoses[index])
      return index;
  return -1;
}

void Motion::propagatePoseUpdate() {
  Pose *pose = dynamic_cast<Pose *>(sender());
  if (pose) {
    int index = computePoseToIndex(pose);
    assert(index != -1);
    emit poseUpdated(index);
  }
}

void Motion::propagateSelection() {
  Pose *pose = dynamic_cast<Pose *>(sender());
  if (pose) {
    int index = computePoseToIndex(pose);
    if (index != -1)
      emit poseSelected(index);
  }
}

void Motion::newPoseAt(int index) {
  if (index >= 0 && index <= mPoses.count()) {
    Pose *pose = new Pose;
    pose->setName(tr("Unnamed %1").arg(mNewPoseCounter++));

    if (mPoses.count() > 0) {
      Pose *referencePose = mPoses[0];
      foreach (MotorTargetState *referenceState, referencePose->states()) {
        pose->createAndAppendState(referenceState->motor());
      }
    }

    if (!hasFixedStep()) {
      const Pose *previousPose = index > 0 ? mPoses.at(index - 1) : NULL;
      const Pose *nextPose = index < mPoses.count() ? mPoses.at(index) : NULL;
      if (previousPose && nextPose)
        pose->setTime(0.5 * (previousPose->time() + nextPose->time()));
      else if (previousPose)
        pose->setTime(previousPose->time());
      else if (nextPose)
        pose->setTime(nextPose->time());
    }

    connect(pose, SIGNAL(updated()), this, SLOT(propagatePoseUpdate()));
    connect(pose, SIGNAL(selected()), this, SLOT(propagateSelection()));
    mPoses.insert(index, pose);
    emit poseInserted(index);

    if (hasFixedStep())
      setFixedStep(fixedStep());
  }
}

void Motion::duplicatePoseAt(int index) {
  if (index >= 0 && index < mPoses.count()) {
    Pose *originalPose = mPoses.at(index);
    Pose *newPose = new Pose(*originalPose);
    QString originalName = originalPose->name();
    if (originalName.contains(QRegularExpression("^.*\\s\\(\\d+\\)$"))) {
      int startIndex = originalName.lastIndexOf("(");
      int endIndex = originalName.lastIndexOf(")");
      QString numberString = originalName.mid(startIndex + 1, endIndex - startIndex - 1);
      newPose->setName(QString("%1(%2)").arg(originalName.left(startIndex)).arg(numberString.toInt() + 1));

    } else
      newPose->setName(QString("%1 (1)").arg(originalPose->name()));

    connect(newPose, SIGNAL(updated()), this, SLOT(propagatePoseUpdate()));
    connect(newPose, SIGNAL(selected()), this, SLOT(propagateSelection()));
    mPoses.insert(index + 1, newPose);
    emit poseInserted(index + 1);

    if (hasFixedStep())
      setFixedStep(fixedStep());
  }
}

void Motion::deletePoseAt(int index) {
  if (index >= 0 && index < mPoses.count()) {
    Pose *pose = mPoses.takeAt(index);
    delete pose;
    emit poseDeleted(index);

    if (hasFixedStep())
      setFixedStep(fixedStep());
  }
}

void Motion::swapPoseWithNext(int index) {
  if (index >= 0 && index < mPoses.count() - 1) {
    Pose *currentPose = mPoses.at(index);
    Pose *nextPose = mPoses.at(index + 1);

    int currentTime = currentPose->time();
    int nextTime = nextPose->time();

    currentPose->setTime(nextTime);
    nextPose->setTime(currentTime);

    mPoses.takeAt(index);
    emit poseDeleted(index);
    mPoses.insert(index + 1, currentPose);
    emit poseInserted(index + 1);
  }
}

void Motion::deleteStatesByMotorName(const QString &motorName) {
  foreach (Pose *pose, mPoses) {
    int index = pose->computeIndexOfStateByMotorName(motorName);
    pose->deleteStateAt(index);
  }
}

bool Motion::isSomeStateDefinedByMotorName(const QString &motorName) const {
  foreach (Pose *pose, mPoses) {
    MotorTargetState *state = pose->findStateByMotorName(motorName);
    if (state->isDefined())
      return true;
  }
  return false;
}

void Motion::addNewMotor(const QString &motorName) {
  Motor *motor = NULL;
  foreach (Motor *sm, MotionGlobalSettings::availableMotorList()) {
    if (sm->name() == motorName) {
      motor = sm;
      break;
    }
  }

  assert(motor);

  foreach (Pose *pose, mPoses)
    pose->addNewMotor(motor);
}
