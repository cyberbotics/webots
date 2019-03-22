// Copyright 1996-2019 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbContactProperties.hpp"

#include "WbFieldChecker.hpp"
#include "WbSoundEngine.hpp"
#include "WbUrl.hpp"
#include "WbWorld.hpp"

void WbContactProperties::init() {
  mMaterial1 = findSFString("material1");
  mMaterial2 = findSFString("material2");
  mCoulombFriction = findMFDouble("coulombFriction");
  mFrictionRotation = findSFVector2("frictionRotation");
  mBounce = findSFDouble("bounce");
  mBounceVelocity = findSFDouble("bounceVelocity");
  mForceDependentSlip = findMFDouble("forceDependentSlip");
  mSoftErp = findSFDouble("softERP");
  mSoftCfm = findSFDouble("softCFM");
  mBumpSound = findSFString("bumpSound");
  mRollSound = findSFString("rollSound");
  mSlideSound = findSFString("slideSound");

  mBumpSoundClip = NULL;
  mRollSoundClip = NULL;
  mSlideSoundClip = NULL;
}

WbContactProperties::WbContactProperties(WbTokenizer *tokenizer) : WbBaseNode("ContactProperties", tokenizer) {
  init();
}

WbContactProperties::WbContactProperties(const WbContactProperties &other) : WbBaseNode(other) {
  init();
}

WbContactProperties::WbContactProperties(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbContactProperties::~WbContactProperties() {
}
void WbContactProperties::preFinalize() {
  WbBaseNode::preFinalize();

  updateCoulombFriction();
  updateBounce();
  updateBounceVelocity();
  updateSoftCfm();
  updateSoftErp();
  updateBumpSound();
  updateRollSound();
  updateSlideSound();
}

void WbContactProperties::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mMaterial1, &WbSFString::changed, this, &WbContactProperties::valuesChanged);
  connect(mMaterial2, &WbSFString::changed, this, &WbContactProperties::valuesChanged);
  connect(mCoulombFriction, &WbSFDouble::changed, this, &WbContactProperties::updateCoulombFriction);
  connect(mFrictionRotation, &WbSFVector2::changed, this, &WbContactProperties::updateFrictionRotation);
  connect(mBounce, &WbSFDouble::changed, this, &WbContactProperties::updateBounce);
  connect(mBounceVelocity, &WbSFDouble::changed, this, &WbContactProperties::updateBounceVelocity);
  connect(mForceDependentSlip, &WbSFDouble::changed, this, &WbContactProperties::updateForceDependentSlip);
  connect(mSoftErp, &WbSFDouble::changed, this, &WbContactProperties::updateSoftErp);
  connect(mSoftCfm, &WbSFDouble::changed, this, &WbContactProperties::updateSoftCfm);
  connect(mBumpSound, &WbSFString::changed, this, &WbContactProperties::updateBumpSound);
  connect(mRollSound, &WbSFString::changed, this, &WbContactProperties::updateRollSound);
  connect(mSlideSound, &WbSFString::changed, this, &WbContactProperties::updateSlideSound);
  connect(this, &WbContactProperties::needToEnableBodies, this, &WbContactProperties::enableBodies);
}

void WbContactProperties::updateCoulombFriction() {
  const int nbElements = mCoulombFriction->size();
  if (nbElements < 1 || nbElements > 4) {
    warn(tr("'coulombFriction' must have between one and four elements"));
    return;
  }

  for (int c = 0; c < nbElements; ++c) {
    const double cf = mCoulombFriction->item(c);
    if (cf != -1.0 && cf < 0.0) {
      warn(tr("If not set to -1 (infinity), 'coulombFriction' must be non-negative. Field value reset to 1"));
      mCoulombFriction->setItem(c, 1.0);
      return;
    }
  }

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void WbContactProperties::updateFrictionRotation() {
  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void WbContactProperties::updateBounce() {
  if (WbFieldChecker::checkDoubleInRangeWithIncludedBounds(this, mBounce, 0.0, 1.0, 0.5))
    return;

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void WbContactProperties::updateBounceVelocity() {
  if (WbFieldChecker::checkDoubleIsNonNegative(this, mBounceVelocity, 0.01))
    return;

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void WbContactProperties::updateForceDependentSlip() {
  const int nbElements = mForceDependentSlip->size();
  if (nbElements < 1 || nbElements > 4) {
    warn(tr("'forceDependentSlip' must have between one and four elements"));
    return;
  }

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void WbContactProperties::updateSoftCfm() {
  if (WbFieldChecker::checkDoubleIsNonNegative(this, mSoftCfm, 0.001))
    return;
  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void WbContactProperties::updateSoftErp() {
  if (WbFieldChecker::checkDoubleInRangeWithIncludedBounds(this, mSoftErp, 0.0, 1.0, 0.2))
    return;

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void WbContactProperties::updateBumpSound() {
  const QString &sound = mBumpSound->value();
  if (sound.isEmpty())
    mBumpSoundClip = NULL;
  else
    mBumpSoundClip = WbSoundEngine::sound(WbUrl::computePath(this, "bumpSound", sound));
  WbSoundEngine::clearAllContactSoundSources();
}

void WbContactProperties::updateRollSound() {
  const QString &sound = mRollSound->value();
  if (sound.isEmpty())
    mRollSoundClip = NULL;
  else
    mRollSoundClip = WbSoundEngine::sound(WbUrl::computePath(this, "rollSound", sound));
  WbSoundEngine::clearAllContactSoundSources();
}

void WbContactProperties::updateSlideSound() {
  const QString &sound = mSlideSound->value();
  if (sound.isEmpty())
    mSlideSoundClip = NULL;
  else
    mSlideSoundClip = WbSoundEngine::sound(WbUrl::computePath(this, "slideSound", sound));
  WbSoundEngine::clearAllContactSoundSources();
}

void WbContactProperties::enableBodies() {
  // when a value is changed, we need to re-enable the bodies,
  // otherwise the new value is not applied in WbSimulationCluster::fillSurfaceParameters
  WbWorld::instance()->awake();
}
