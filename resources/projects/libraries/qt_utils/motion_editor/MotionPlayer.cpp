#include "MotionPlayer.hpp"

#include <devices/Motor.hpp>
#include "Motion.hpp"
#include "MotorTargetState.hpp"
#include "Pose.hpp"

#include <webots/robot.h>

#include <cassert>

using namespace webotsQtUtils;

static double modulus(double a, double b) {
  int result = static_cast<int>(a / b);
  return a - static_cast<double>(result) * b;
}

MotionPlayer::MotionPlayer() :
  mMotion(NULL),
  mStartingTime(0),
  mPoseTimeOffset(0),
  mMotionDuration(0),
  mLoop(false),
  mReverse(false) {
}

MotionPlayer::~MotionPlayer() {
}

void MotionPlayer::play(Motion *motion, int poseIndex, bool reverse) {
  mMotion = motion;
  mReverse = reverse;

  mStartingTime = getTime();

  QList<Pose *> poses = mMotion->poses();
  assert(poseIndex >= 0 && poseIndex < poses.size());
  mPoseTimeOffset = poses[poseIndex]->time();

  updateMotionDuration();
}

void MotionPlayer::stop() {
  mMotion = NULL;
  mReverse = false;

  mStartingTime = 0;
  mPoseTimeOffset = 0;
  mMotionDuration = 0;
}

void MotionPlayer::writeActuators() {
  if (!isPlaying())
    return;

  int currentTime = getTime() - mStartingTime;
  if (mReverse)
    currentTime += mMotionDuration - mPoseTimeOffset;
  else
    currentTime += mPoseTimeOffset;

  if (mLoop)
    currentTime = modulus(currentTime, mMotionDuration);

  if (mReverse)
    currentTime = mMotionDuration - currentTime;

  // find the pose directly before and after the current time
  Pose *beforePose = NULL;
  Pose *afterPose = NULL;
  foreach (Pose *pose, mMotion->poses()) {
    int poseTime = pose->time();
    if (poseTime <= currentTime) {
      // cppcheck-suppress knownConditionTrueFalse
      if (!beforePose || (poseTime < currentTime && poseTime > beforePose->time()))
        beforePose = pose;
    } else {
      // cppcheck-suppress knownConditionTrueFalse
      if (!afterPose || (poseTime > currentTime && poseTime < afterPose->time()))
        afterPose = pose;
    }
  }

  // apply the found poses to the motor
  // cppcheck-suppress nullPointerRedundantCheck
  if (beforePose && afterPose) {
    assert(beforePose->states().count() == afterPose->states().count());

    int count = beforePose->states().count();
    int beforeTime = beforePose->time();
    int afterTime = afterPose->time();
    int deltaTime = afterTime - beforeTime;
    double ratio = ((double)currentTime - beforeTime) / deltaTime;
    if (ratio < 0.5)
      beforePose->select();
    else
      afterPose->select();

    for (int i = 0; i < count; i++) {
      MotorTargetState *beforeState = beforePose->states()[i];
      MotorTargetState *afterState = afterPose->states()[i];

      assert(beforeState->motor()->tag() == afterState->motor()->tag());

      Motor *motor = beforeState->motor();

      if (beforeState->isDefined() && afterState->isDefined()) {
        double beforeValue = beforeState->value();
        double afterValue = afterState->value();
        double deltaValue = afterValue - beforeValue;

        double value = ratio * deltaValue + beforeValue;  // linear interpolation
        motor->setPosition(value);
      }
      // TODO: the following code could not work if the
      //       controller sampling rate is lower than the
      //       motion sampling rate
      else if (beforeState->isDefined()) {
        motor->setPosition(beforeState->value());
        beforePose->select();
      }
    }
  }
  // bounds management
  else if (beforePose || afterPose) {
    Pose *pose = beforePose ? beforePose : afterPose;
    // cppcheck-suppress nullPointerRedundantCheck
    pose->select();

    // cppcheck-suppress nullPointerRedundantCheck
    int count = pose->states().count();

    for (int i = 0; i < count; i++) {
      MotorTargetState *state = pose->states()[i];
      Motor *motor = state->motor();
      motor->setPosition(state->value());
    }
  }

  if (!mLoop && !mReverse && !afterPose)
    emit motionEnded();
  else if (!mLoop && mReverse && !beforePose)
    emit motionEnded();
}

int MotionPlayer::getTime() const {
  return 1000 * wb_robot_get_time();
}

void MotionPlayer::updateMotionDuration() {
  mMotionDuration = 0;
  if (!mMotion)
    return;

  foreach (Pose *pose, mMotion->poses()) {
    if (pose->time() > mMotionDuration)
      mMotionDuration = pose->time();
  }
}
