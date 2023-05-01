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

#include "WbRadar.hpp"

#include "WbAffinePlane.hpp"
#include "WbDataStream.hpp"
#include "WbFieldChecker.hpp"
#include "WbObjectDetection.hpp"
#include "WbRandom.hpp"
#include "WbSensor.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QVector>

#include <cassert>

class WbRadarTarget : public WbObjectDetection {
public:
  WbRadarTarget(WbRadar *radar, WbSolid *solidTarget, const bool needToCheckCollision, const double maxRange) :
    WbObjectDetection(radar, solidTarget, needToCheckCollision ? WbObjectDetection::ONE_RAY : WbObjectDetection::NONE, maxRange,
                      radar->horizontalFieldOfView()) {
    mTargetDistance = 0.0;
    mReceivedPower = 0.0;
    mSpeed = 0.0;
    mAzimuth = 0.0;
  };

  virtual ~WbRadarTarget() {}

  double targetDistance() const { return mTargetDistance; }
  double receivedPower() const { return mReceivedPower; }
  double speed() const { return mSpeed; }
  double azimuth() const { return mAzimuth; }
  void setTargetDistance(double distance) { mTargetDistance = distance; }
  void setReceivedPower(double receivedPower) { mReceivedPower = receivedPower; }
  void setSpeed(double speed) { mSpeed = speed; }
  void setAzimuth(double azimuth) { mAzimuth = azimuth; }

protected:
  double distance() override { return objectRelativePosition().length(); }

  double mTargetDistance;
  double mReceivedPower;
  double mSpeed;
  double mAzimuth;
};

void WbRadar::init() {
  mMinRange = findSFDouble("minRange");
  mMaxRange = findSFDouble("maxRange");
  mHorizontalFieldOfView = findSFDouble("horizontalFieldOfView");
  mVerticalFieldOfView = findSFDouble("verticalFieldOfView");
  mMinAbsoluteRadialSpeed = findSFDouble("minAbsoluteRadialSpeed");
  mMinRadialSpeed = findSFDouble("minRadialSpeed");
  mMaxRadialSpeed = findSFDouble("maxRadialSpeed");
  mCellDistance = findSFDouble("cellDistance");
  mCellSpeed = findSFDouble("cellSpeed");
  mRangeNoise = findSFDouble("rangeNoise");
  mSpeedNoise = findSFDouble("speedNoise");
  mAngularNoise = findSFDouble("angularNoise");
  mAntennaGain = findSFDouble("antennaGain");
  mFrequency = findSFDouble("frequency");
  mTransmittedPower = findSFDouble("transmittedPower");
  mOcclusion = findSFBool("occlusion");
  mMinDetectableSignal = findSFDouble("minDetectableSignal");

  mSensorElapsedTime = 0;
  mSensor = NULL;

  mTransform = NULL;
  mRenderable = NULL;
  mMesh = NULL;
  mMaterial = NULL;

  mTransmittedPowerInW = 0.0;
  mRealGain = 0.0;
  mReceivedPowerThreshold = 0.0;
  mReceivedPowerFactor = 1.0;
}

WbRadar::WbRadar(WbTokenizer *tokenizer) : WbSolidDevice("Radar", tokenizer) {
  init();
}

WbRadar::WbRadar(const WbRadar &other) : WbSolidDevice(other) {
  init();
}

WbRadar::WbRadar(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbRadar::~WbRadar() {
  delete mSensor;
  if (areWrenObjectsInitialized()) {
    wr_node_delete(WR_NODE(mTransform));
    wr_node_delete(WR_NODE(mRenderable));
    wr_material_delete(mMaterial);
    wr_static_mesh_delete(mMesh);
  }
  qDeleteAll(mRadarTargets);
  mRadarTargets.clear();
}

void WbRadar::preFinalize() {
  WbSolidDevice::preFinalize();

  mSensor = new WbSensor();
}

void WbRadar::postFinalize() {
  WbSolidDevice::postFinalize();

  connect(mMinRange, &WbSFDouble::changed, this, &WbRadar::updateMinRange);
  connect(mMaxRange, &WbSFDouble::changed, this, &WbRadar::updateMaxRange);
  connect(mHorizontalFieldOfView, &WbSFDouble::changed, this, &WbRadar::updateHorizontalFieldOfView);
  connect(mVerticalFieldOfView, &WbSFDouble::changed, this, &WbRadar::updateVerticalFieldOfView);
  connect(mMinAbsoluteRadialSpeed, &WbSFDouble::changed, this, &WbRadar::updateMinAbsoluteRadialSpeed);
  connect(mMinRadialSpeed, &WbSFDouble::changed, this, &WbRadar::updateMinAndMaxRadialSpeed);
  connect(mMaxRadialSpeed, &WbSFDouble::changed, this, &WbRadar::updateMinAndMaxRadialSpeed);
  connect(mCellDistance, &WbSFDouble::changed, this, &WbRadar::updateCellDistance);
  connect(mCellSpeed, &WbSFDouble::changed, this, &WbRadar::updateCellSpeed);
  connect(mRangeNoise, &WbSFDouble::changed, this, &WbRadar::updateRangeNoise);
  connect(mSpeedNoise, &WbSFDouble::changed, this, &WbRadar::updateSpeedNoise);
  connect(mAngularNoise, &WbSFDouble::changed, this, &WbRadar::updateAngularNoise);
  connect(mFrequency, &WbSFDouble::changed, this, &WbRadar::updateFrequency);
  connect(mAntennaGain, &WbSFDouble::changed, this, &WbRadar::updateAntennaGain);
  connect(mTransmittedPower, &WbSFDouble::changed, this, &WbRadar::updateTransmittedPower);
  connect(mMinDetectableSignal, &WbSFDouble::changed, this, &WbRadar::updateMinDetectableSignal);
  connect(mSensor, &WbSensor::stateChanged, this, &WbRadar::applyFrustumToWren);
  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::optionalRenderingChanged, this,
          &WbRadar::updateOptionalRendering);

  updateMinRange();
  updateMaxRange();
  updateHorizontalFieldOfView();
  updateVerticalFieldOfView();
  updateMinAbsoluteRadialSpeed();
  updateMinAndMaxRadialSpeed();
  updateCellDistance();
  updateCellSpeed();
  updateRangeNoise();
  updateSpeedNoise();
  updateAngularNoise();
  updateFrequency();
  updateAntennaGain();
  updateTransmittedPower();
  updateMinDetectableSignal();
}

void WbRadar::updateOptionalRendering(int option) {
  if (option == WbWrenRenderingContext::VF_RADAR_FRUSTUMS && areWrenObjectsInitialized()) {
    applyFrustumToWren();
    wr_node_set_visible(WR_NODE(mTransform), WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(option));
  }
}

void WbRadar::updateMinRange() {
  WbFieldChecker::resetDoubleIfNegative(this, mMinRange, 0.0);
  if (mMaxRange->value() <= mMinRange->value()) {
    if (mMaxRange->value() == 0.0) {
      double newMaxRange = mMinRange->value() + 1.0;
      parsingWarn(tr("'minRange' is greater or equal to 'maxRange'. Setting 'maxRange' to %1.").arg(newMaxRange));
      mMaxRange->setValueNoSignal(newMaxRange);
    } else {
      double newMinRange = mMaxRange->value() - 1.0;
      if (newMinRange < 0.0)
        newMinRange = 0.0;
      parsingWarn(tr("'minRange' is greater or equal to 'maxRange'. Setting 'minRange' to %1.").arg(newMinRange));
      mMinRange->setValueNoSignal(newMinRange);
    }
    return;
  }
  if (areWrenObjectsInitialized())
    applyFrustumToWren();
}

void WbRadar::updateMaxRange() {
  WbFieldChecker::resetDoubleIfNegative(this, mMaxRange, mMinRange->value() + 1.0);

  if (mMaxRange->value() <= mMinRange->value()) {
    double newMaxRange = mMinRange->value() + 1.0;
    parsingWarn(tr("'maxRange' is less or equal to 'minRange'. Setting 'maxRange' to %1.").arg(newMaxRange));
    mMaxRange->setValueNoSignal(newMaxRange);
    return;
  }
  if (areWrenObjectsInitialized())
    applyFrustumToWren();
}

void WbRadar::updateHorizontalFieldOfView() {
  WbFieldChecker::resetDoubleIfNotInRangeWithExcludedBounds(this, mHorizontalFieldOfView, 0.0, M_PI, 0.78);
  if (areWrenObjectsInitialized())
    applyFrustumToWren();
}

void WbRadar::updateVerticalFieldOfView() {
  WbFieldChecker::resetDoubleIfNotInRangeWithExcludedBounds(this, mVerticalFieldOfView, 0.0, M_PI_2, 0.1);
  if (areWrenObjectsInitialized())
    applyFrustumToWren();
}

void WbRadar::updateMinAbsoluteRadialSpeed() {
  WbFieldChecker::resetDoubleIfNegative(this, mMinAbsoluteRadialSpeed, 0.0);
}

void WbRadar::updateMinAndMaxRadialSpeed() {
  if (mMinRadialSpeed->value() == 0.0 && mMaxRadialSpeed->value() == 0.0)
    // no limits
    return;

  if (mMaxRadialSpeed->value() <= mMinRadialSpeed->value()) {
    double newMaxRadialSpeed = mMinRadialSpeed->value() + 1.0;
    parsingWarn(
      tr("'maxRadialSpeed' is less than or equal to 'minRadialSpeed'. Setting 'maxRadialSpeed' to %1.").arg(newMaxRadialSpeed));
    mMaxRadialSpeed->setValueNoSignal(newMaxRadialSpeed);
    return;
  }
}

void WbRadar::updateCellDistance() {
  WbFieldChecker::resetDoubleIfNegative(this, mCellDistance, 0.0);
}

void WbRadar::updateCellSpeed() {
  WbFieldChecker::resetDoubleIfNegative(this, mCellSpeed, 0.0);
}

void WbRadar::updateRangeNoise() {
  WbFieldChecker::resetDoubleIfNegative(this, mRangeNoise, 0.0);
}

void WbRadar::updateSpeedNoise() {
  WbFieldChecker::resetDoubleIfNegative(this, mSpeedNoise, 0.0);
}

void WbRadar::updateAngularNoise() {
  WbFieldChecker::resetDoubleIfNegative(this, mAngularNoise, 0.0);
}

void WbRadar::updateFrequency() {
  WbFieldChecker::resetDoubleIfNegative(this, mFrequency, 24.0);
  updateReceivedPowerFactor();
}

void WbRadar::updateTransmittedPower() {
  // convert transmittedPower from dBm to W
  mTransmittedPowerInW = 0.001 * pow(10.0, mTransmittedPower->value() / 10.0);
  updateReceivedPowerFactor();
}

void WbRadar::updateAntennaGain() {
  // convert antennaGain from dBi to to real gain
  mRealGain = pow(10.0, mAntennaGain->value() / 10.0);
  updateReceivedPowerFactor();
}

void WbRadar::updateMinDetectableSignal() {
  // convert minDetectableSignal from dBm into W
  mReceivedPowerThreshold = 0.001 * pow(10.0, mMinDetectableSignal->value() / 10.0);
}

void WbRadar::updateReceivedPowerFactor() {
  // compute the received power factor
  //(part of the received power formula only dependent on the radar caracteristics)
  mReceivedPowerFactor = mTransmittedPowerInW * pow(mRealGain, 2.0) * pow(wavelength(), 2.0) / pow(4 * M_PI, 3.0);
}

void WbRadar::prePhysicsStep(double ms) {
  WbSolidDevice::prePhysicsStep(ms);

  qDeleteAll(mRadarTargets);
  mRadarTargets.clear();

  if (isPowerOn() && mSensor->isEnabled() && mSensor->needToRefreshInMs(ms) && mOcclusion->value()) {
    // create rays
    computeTargets(false, true);

    if (!mRadarTargets.isEmpty()) {
      // radar or targets could move during physics step
      const QList<dGeomID> &rays = mRadarTargets[0]->geoms();
      if (!rays.isEmpty())
        subscribeToRaysUpdate(rays.first());
    }
  }
}

void WbRadar::postPhysicsStep() {
  WbSolidDevice::postPhysicsStep();

  // delete invalid WbRadarTargets and ODE rays
  // it is preferable to not delete them during the physics step to avoid
  // possible inconsistencies in other clusters
  qDeleteAll(mInvalidRadarTargets);
  mInvalidRadarTargets.clear();
}

void WbRadar::updateRaysSetupIfNeeded() {
  updateTransformForPhysicsStep();

  // compute the radar position, rotation, axis and plane
  const WbVector3 radarPosition = position();
  const WbMatrix3 radarRotation = rotationMatrix();
  const WbVector3 radarAxis = radarRotation * WbVector3(1.0, 0.0, 0.0);
  const WbAffinePlane radarPlane(radarRotation * WbVector3(0.0, 0.0, 1.0), radarAxis);
  const WbAffinePlane *frustumPlanes =
    WbObjectDetection::computeFrustumPlanes(this, verticalFieldOfView(), horizontalFieldOfView(), maxRange(), true);
  foreach (WbRadarTarget *target, mRadarTargets) {
    target->object()->updateTransformForPhysicsStep();
    if (!target->recomputeRayDirection(frustumPlanes) ||
        !setTargetProperties(radarPosition, radarRotation, radarAxis, radarPlane, target)) {
      mRadarTargets.removeAll(target);
      mInvalidRadarTargets.append(target);
    }
  }

  delete[] frustumPlanes;
}

void WbRadar::rayCollisionCallback(dGeomID geom, WbSolid *collidingSolid, double depth) {
  foreach (WbRadarTarget *target, mRadarTargets) {
    // check if this target is the one that collides
    if (target->contains(geom)) {
      // make sure the colliding solid is not the target itself
      if (target->object() != collidingSolid && !target->object()->solidChildren().contains(collidingSolid))
        target->setCollided(geom, depth);
      return;
    }
  }
}

void WbRadar::createWrenObjects() {
  mTransform = wr_transform_new();
  wr_node_set_visible(WR_NODE(mTransform), false);

  mMaterial = wr_phong_material_new();
  wr_material_set_default_program(mMaterial, WbWrenShaders::lineSetShader());

  mRenderable = wr_renderable_new();
  wr_renderable_set_visibility_flags(mRenderable, WbWrenRenderingContext::VF_RADAR_FRUSTUMS);
  wr_renderable_set_drawing_mode(mRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);
  wr_renderable_set_material(mRenderable, mMaterial, NULL);

  WbSolidDevice::createWrenObjects();

  wr_transform_attach_child(mTransform, WR_NODE(mRenderable));
  wr_transform_attach_child(wrenNode(), WR_NODE(mTransform));

  applyFrustumToWren();
  updateOptionalRendering(WbWrenRenderingContext::VF_RADAR_FRUSTUMS);
}

void WbRadar::handleMessage(QDataStream &stream) {
  unsigned char command;
  short refreshRate;
  stream >> command;

  switch (command) {
    case C_SET_SAMPLING_PERIOD:
      stream >> refreshRate;
      mSensor->setRefreshRate(refreshRate);
      return;
    default:
      assert(0);
  }
}

void WbRadar::writeConfigure(WbDataStream &stream) {
  mSensor->connectToRobotSignal(robot());

  stream << tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (double)minRange();
  stream << (double)maxRange();
  stream << (double)horizontalFieldOfView();
  stream << (double)verticalFieldOfView();
}

void WbRadar::writeAnswer(WbDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << tag();
    stream << (unsigned char)C_RADAR_DATA;
    int numberOfTargets = mRadarTargets.size();
    stream << (int)numberOfTargets;
    for (int i = 0; i < numberOfTargets; ++i) {
      stream << (double)mRadarTargets.at(i)->targetDistance();
      stream << (double)mRadarTargets.at(i)->receivedPower();
      stream << (double)mRadarTargets.at(i)->speed();
      stream << (double)mRadarTargets.at(i)->azimuth();
    }
    mSensor->resetPendingValue();
  }
}

void WbRadar::computeTargets(bool finalSetup, bool needCollisionDetection) {
  // compute the radar position, rotation, axis and plane
  const WbVector3 radarPosition = position();
  const WbMatrix3 radarRotation = rotationMatrix();
  const WbVector3 radarAxis = radarRotation * WbVector3(1.0, 0.0, 0.0);
  const WbAffinePlane radarPlane(radarRotation * WbVector3(0.0, 0.0, 1.0), radarAxis);
  const WbAffinePlane *frustumPlanes =
    WbObjectDetection::computeFrustumPlanes(this, verticalFieldOfView(), horizontalFieldOfView(), maxRange(), true);

  // loop for each possible target to check if it is visible
  QList<WbSolid *> targets = WbWorld::instance()->radarTargetSolids();
  for (int i = 0; i < targets.size(); i++) {
    WbSolid *target = targets.at(i);
    if (target == this)
      continue;
    // create target
    WbRadarTarget *generatedTarget = new WbRadarTarget(this, target, needCollisionDetection, maxRange());
    if (finalSetup) {
      if (!generatedTarget->isContainedInFrustum(frustumPlanes) ||
          !setTargetProperties(radarPosition, radarRotation, radarAxis, radarPlane, generatedTarget)) {
        delete generatedTarget;
        continue;
      }
    }
    mRadarTargets.append(generatedTarget);
  }

  delete[] frustumPlanes;
}

bool WbRadar::setTargetProperties(const WbVector3 &radarPosition, const WbMatrix3 &radarRotation, const WbVector3 &radarAxis,
                                  const WbAffinePlane &radarPlane, WbRadarTarget *radarTarget) {
  assert(radarTarget);

  const WbVector3 targetPosition = radarTarget->object()->position();
  const WbVector3 targetToRadarVector = targetPosition - radarPosition;

  double distance = radarTarget->objectRelativePosition().length() + mRangeNoise->value() * WbRandom::nextGaussian();

  // check that target is not too close
  if (distance < (minRange() - radarTarget->objectSize().x() / 2.0))
    return false;

  if (distance > maxRange())
    distance = maxRange();
  else if (distance < minRange())
    distance = minRange();

  // compute received power and check if it is above the threshold
  // the received power is converted to dBm after the targets are merged
  // this is done to avoid converting back and forth when merging two targets
  double receivedPower = mReceivedPowerFactor * radarTarget->object()->radarCrossSection() / pow(distance, 4.0);
  if (receivedPower < mReceivedPowerThreshold)
    return false;

  // compute speed using (distance / time).
  WbVector3 targetVelocity =
    (targetPosition - mRadarTargetsPreviousTranslations[radarTarget->object()]) * (1000 / mSensorElapsedTime);
  WbVector3 radarVelocity = (radarPosition - mPreviousRadarPosition) * (1000 / mSensorElapsedTime);
  WbVector3 relativeVelocity = targetVelocity - radarVelocity;

  double relativeSpeed = targetToRadarVector.normalized().dot(relativeVelocity);
  relativeSpeed += mSpeedNoise->value() * WbRandom::nextGaussian();
  // check speed is in the radial speed range
  if (mMinRadialSpeed->value() != 0.0 || mMaxRadialSpeed->value() != 0.0) {
    if (relativeSpeed < mMinRadialSpeed->value())
      return false;
    else if (relativeSpeed > mMaxRadialSpeed->value())
      return false;
  }

  // check that absolute speed is bigger than 'mMinAbsoluteRadialSpeed'
  if (mMinAbsoluteRadialSpeed->value() > 0 && fabs(relativeSpeed) < mMinAbsoluteRadialSpeed->value())
    return false;

  // compute horizontal angle
  WbVector3 projectedTargetToRadarVector = radarPlane.vectorProjection(targetToRadarVector);
  projectedTargetToRadarVector.normalize();
  double azimuth = radarAxis.angle(projectedTargetToRadarVector);
  if (projectedTargetToRadarVector.dot(radarRotation * WbVector3(0.0, 1.0, 0.0)) > 0.0)
    azimuth = -azimuth;

  // checks that azimuth is not out of the detection frustum,
  // this can happen if the object's center is outside but a part of the object is inside.
  // In that case a future improvement would be to adapt the received power.
  if (azimuth > (mHorizontalFieldOfView->value() / 2.0))
    azimuth = mHorizontalFieldOfView->value() / 2.0;
  else if (azimuth < -(mHorizontalFieldOfView->value() / 2.0))
    azimuth = -mHorizontalFieldOfView->value() / 2.0;

  azimuth += mAngularNoise->value() * WbRandom::nextGaussian();

  // update target
  radarTarget->setTargetDistance(distance);
  radarTarget->setReceivedPower(receivedPower);
  radarTarget->setSpeed(relativeSpeed);
  radarTarget->setAzimuth(azimuth);
  return true;
}

bool WbRadar::refreshSensorIfNeeded() {
  if (!isPowerOn() || !mSensor->needToRefresh())
    return false;

  if (!mOcclusion->value())
    // no need of ODE ray collision detection
    // rays can be created at the end of the step when all the body positions
    // are up-to-date
    computeTargets(true, false);
  else
    // post process targets
    removeOccludedTargets();

  if (mCellDistance->value() > 0.0)
    mergeTargets();

  // convert the received power into dBm
  for (int i = 0; i < mRadarTargets.size(); ++i)
    mRadarTargets.at(i)->setReceivedPower(10.0 * log10(mRadarTargets.at(i)->receivedPower() / 0.001));

  mRadarTargetsPreviousTranslations.clear();
  QList<WbSolid *> targets = WbWorld::instance()->radarTargetSolids();
  for (int i = 0; i < targets.size(); ++i) {
    WbSolid *target = targets.at(i);
    if (target != this)
      mRadarTargetsPreviousTranslations.insert(target, target->position());
  }

  // cache the sensor timestep for use in velocity calculations
  mPreviousRadarPosition = position();
  mSensorElapsedTime = mSensor->elapsedTime();
  mSensor->updateTimer();
  return true;
}

void WbRadar::reset(const QString &id) {
  WbSolidDevice::reset(id);

  qDeleteAll(mRadarTargets);
  mRadarTargets.clear();
  qDeleteAll(mInvalidRadarTargets);
  mInvalidRadarTargets.clear();
  mRadarTargetsPreviousTranslations.clear();
}

void WbRadar::removeOccludedTargets() {
  for (int i = mRadarTargets.size() - 1; i >= 0; --i) {
    if (mRadarTargets.at(i)->hasCollided()) {
      delete mRadarTargets[i];
      mRadarTargets.removeAt(i);
    }
  }
}

void WbRadar::mergeTargets(int startingIndex) {
  // Merge the targets recursively if the distance is < 'cellDistance'
  int numberOfTargets = mRadarTargets.size();
  if (startingIndex >= numberOfTargets)
    return;
  double cellDistance = mCellDistance->value();
  double cellSpeed = mCellSpeed->value();
  bool targetMerged = false;
  int i = 0;
  for (i = startingIndex; i < numberOfTargets - 1; ++i) {
    QVector<int> targetsToMerge;
    for (int j = i + 1; j < numberOfTargets; ++j) {
      if ((fabs(mRadarTargets.at(i)->targetDistance() - mRadarTargets.at(j)->targetDistance()) < cellDistance) &&
          ((cellSpeed <= 0.0) || (fabs(mRadarTargets.at(i)->speed() - mRadarTargets.at(j)->speed()) < cellSpeed))) {
        targetsToMerge.append(j);
        targetMerged = true;
      }
    }
    if (targetMerged) {
      // use a weighted average (weight is the received power) to compute resulting target
      double resultingReceivedPower = mRadarTargets.at(i)->receivedPower();
      double resultingAzimuth = mRadarTargets.at(i)->azimuth() * resultingReceivedPower;
      double resultingSpeed = mRadarTargets.at(i)->speed() * resultingReceivedPower;
      double resultingDistance = mRadarTargets.at(i)->targetDistance() * resultingReceivedPower;
      for (int j = 0; j < targetsToMerge.size(); ++j) {
        resultingReceivedPower += mRadarTargets.at(targetsToMerge.at(j))->receivedPower();
        resultingAzimuth +=
          mRadarTargets.at(targetsToMerge.at(j))->azimuth() * mRadarTargets.at(targetsToMerge.at(j))->receivedPower();
        resultingSpeed +=
          mRadarTargets.at(targetsToMerge.at(j))->speed() * mRadarTargets.at(targetsToMerge.at(j))->receivedPower();
        resultingDistance +=
          mRadarTargets.at(targetsToMerge.at(j))->targetDistance() * mRadarTargets.at(targetsToMerge.at(j))->receivedPower();
      }
      resultingAzimuth /= resultingReceivedPower;
      resultingSpeed /= resultingReceivedPower;
      resultingDistance /= resultingReceivedPower;
      mRadarTargets[i]->setReceivedPower(resultingReceivedPower);
      mRadarTargets[i]->setAzimuth(resultingAzimuth);
      mRadarTargets[i]->setSpeed(resultingSpeed);
      mRadarTargets[i]->setTargetDistance(resultingDistance);
      // remove the merged target
      for (int j = targetsToMerge.size() - 1; j >= 0; --j) {
        delete mRadarTargets[targetsToMerge.at(j)];
        mRadarTargets.removeAt(targetsToMerge.at(j));
      }
      break;
    }
  }
  if (targetMerged)
    mergeTargets(i);
}

static void addVertex(QVector<float> &vertices, float x, float y, float z) {
  vertices.push_back(x);
  vertices.push_back(y);
  vertices.push_back(z);
}

// (Re)creates the blue or gray (radar disabled) frustum if needed
void WbRadar::applyFrustumToWren() {
  // recreates the frustum if it already exists
  if (mMesh) {
    wr_static_mesh_delete(mMesh);
    mMesh = NULL;
  }

  if (!WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_RADAR_FRUSTUMS))
    return;

  const float disabledColor[3] = {0.5f, 0.5f, 0.5f};
  const float enabledColor[3] = {0.0f, 0.0f, 1.0f};
  if (mSensor->isEnabled() && mSensor->isFirstValueReady())
    wr_phong_material_set_color(mMaterial, enabledColor);
  else
    wr_phong_material_set_color(mMaterial, disabledColor);

  const float sinH = sinf(horizontalFieldOfView() / 2.0f);
  const float cosH = cosf(horizontalFieldOfView() / 2.0f);
  const float sinV = sinf(verticalFieldOfView() / 2.0f);
  const float cosV = cosf(verticalFieldOfView() / 2.0f);
  const float factorX = cosV * cosH;
  const float factorY = cosV * sinH;
  const float factorZ = sinV;
  const float maxX = maxRange() * factorX;
  const float maxZ = maxRange() * factorZ;
  const float maxY = maxRange() * factorY;
  const float minX = minRange() * factorX;
  const float minZ = minRange() * factorZ;
  const float minY = minRange() * factorY;

  // create the outlines
  const int steps = 24;
  QVector<float> vertices;
  vertices.reserve(3 * (10 + 16 * steps));

  addVertex(vertices, 0, 0, 0);
  addVertex(vertices, minRange(), 0, 0);
  addVertex(vertices, minX, minY, -minZ);
  addVertex(vertices, maxX, maxY, -maxZ);
  addVertex(vertices, minX, -minY, -minZ);
  addVertex(vertices, maxX, -maxY, -maxZ);
  addVertex(vertices, minX, minY, minZ);
  addVertex(vertices, maxX, maxY, maxZ);
  addVertex(vertices, minX, -minY, minZ);
  addVertex(vertices, maxX, -maxY, maxZ);

  // create top and bottom margin
  const float ranges[2] = {static_cast<float>(minRange()), static_cast<float>(maxRange())};
  float x1, y1, z1, x2, y2, z2;
  for (int j = 0; j < steps; ++j) {
    const float angle1 = horizontalFieldOfView() / 2.0f - j * horizontalFieldOfView() / steps;
    const float angle2 = horizontalFieldOfView() / 2.0f - (j + 1) * horizontalFieldOfView() / steps;
    const float factorX1 = cosV * cosf(angle1);
    const float factorX2 = cosV * cosf(angle2);
    const float factorY1 = cosV * sinf(angle1);
    const float factorY2 = cosV * sinf(angle2);
    for (int k = 0; k < 2; ++k) {
      const float range = ranges[k];
      x1 = range * factorX1;
      y1 = range * factorY1;
      z1 = range * sinV;
      x2 = range * factorX2;
      y2 = range * factorY2;
      z2 = range * sinV;
      // top
      addVertex(vertices, x1, y1, z1);
      addVertex(vertices, x2, y2, z2);
      // bottom
      addVertex(vertices, x1, y1, -z1);
      addVertex(vertices, x2, y2, -z2);
    }
  }

  // create right and left margin
  for (int j = 0; j < steps; ++j) {
    const float angle1 = verticalFieldOfView() / 2.0f - j * verticalFieldOfView() / steps;
    const float angle2 = verticalFieldOfView() / 2.0f - (j + 1) * verticalFieldOfView() / steps;
    const float factorX1 = cosf(angle1) * cosH;
    const float factorX2 = cosf(angle2) * cosH;
    const float factorY1 = cosf(angle1) * sinH;
    const float factorY2 = cosf(angle2) * sinH;
    const float factorZ1 = sinf(angle1);
    const float factorZ2 = sinf(angle2);
    for (int k = 0; k < 2; ++k) {
      const float range = ranges[k];
      x1 = range * factorX1;
      z1 = range * factorZ1;
      y1 = range * factorY1;
      x2 = range * factorX2;
      z2 = range * factorZ2;
      y2 = range * factorY2;
      // right
      addVertex(vertices, x1, -y1, z1);
      addVertex(vertices, x2, -y2, z2);
      // left
      addVertex(vertices, x1, y1, z1);
      addVertex(vertices, x2, y2, z2);
    }
  }

  mMesh = wr_static_mesh_line_set_new(vertices.size() / 3, vertices.data(), NULL);
  wr_renderable_set_mesh(mRenderable, WR_MESH(mMesh));
}
