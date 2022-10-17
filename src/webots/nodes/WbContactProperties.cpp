// Copyright 1996-2022 Cyberbotics Ltd.
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

#include "WbDownloader.hpp"
#include "WbFieldChecker.hpp"
#include "WbNetwork.hpp"
#include "WbSoundEngine.hpp"
#include "WbUrl.hpp"
#include "WbWorld.hpp"

static const QString gUrlNames[3] = {"bumpSound", "rollSound", "slideSound"};

void WbContactProperties::init() {
  mMaterial1 = findSFString("material1");
  mMaterial2 = findSFString("material2");
  mCoulombFriction = findMFDouble("coulombFriction");
  mFrictionRotation = findSFVector2("frictionRotation");
  mRollingFriction = findSFVector3("rollingFriction");
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
  for (int i = 0; i < 3; ++i)
    mDownloader[i] = NULL;
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

void WbContactProperties::downloadAsset(const QString &url, int index) {
  if (url.isEmpty())
    return;

  const QString &completeUrl = WbUrl::computePath(this, gUrlNames[index], url);
  if (!WbUrl::isWeb(completeUrl) || WbNetwork::instance()->isCachedWithMapUpdate(completeUrl))
    return;

  if (mDownloader[index] != NULL)
    delete mDownloader[index];
  mDownloader[index] = new WbDownloader(this);
  if (isPostFinalizedCalled()) {
    void (WbContactProperties::*callback)(void);
    callback = index == 0 ? &WbContactProperties::updateBumpSound :
                            (index == 1 ? &WbContactProperties::updateRollSound : &WbContactProperties::updateSlideSound);
    connect(mDownloader[index], &WbDownloader::complete, this, callback);
  }

  mDownloader[index]->download(QUrl(completeUrl));
}

void WbContactProperties::downloadAssets() {
  downloadAsset(mBumpSound->value(), 0);
  downloadAsset(mRollSound->value(), 1);
  downloadAsset(mSlideSound->value(), 2);
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
  connect(mRollingFriction, &WbSFVector3::changed, this, &WbContactProperties::updateRollingFriction);
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
    parsingWarn(tr("'coulombFriction' must have between one and four elements"));
    return;
  }

  for (int c = 0; c < nbElements; ++c) {
    const double cf = mCoulombFriction->item(c);
    if (cf != -1.0 && cf < 0.0) {
      parsingWarn(tr("If not set to -1 (infinity), 'coulombFriction' must be non-negative. Field value reset to 1"));
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

void WbContactProperties::updateRollingFriction() {
  const WbVector3 &rf = mRollingFriction->value();
  if ((rf[0] != -1.0 && rf[0] < 0.0) || (rf[1] != -1.0 && rf[1] < 0.0) || (rf[2] != -1.0 && rf[2] < 0.0)) {
    parsingWarn(tr("'rollingFriction' values must be positive or -1.0. Field value reset to 0 0 0."));
    mRollingFriction->setValue(0, 0, 0);
    return;
  }

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void WbContactProperties::updateBounce() {
  if (WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mBounce, 0.0, 1.0, 0.5))
    return;

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void WbContactProperties::updateBounceVelocity() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mBounceVelocity, 0.01))
    return;

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void WbContactProperties::updateForceDependentSlip() {
  const int nbElements = mForceDependentSlip->size();
  if (nbElements < 1 || nbElements > 4) {
    parsingWarn(tr("'forceDependentSlip' must have between one and four elements"));
    return;
  }

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void WbContactProperties::updateSoftCfm() {
  if (WbFieldChecker::resetDoubleIfNonPositive(this, mSoftCfm, 0.001))
    return;
  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void WbContactProperties::updateSoftErp() {
  if (WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mSoftErp, 0.0, 1.0, 0.2))
    return;

  if (areOdeObjectsCreated())
    emit valuesChanged();

  emit needToEnableBodies();
}

void WbContactProperties::loadSound(int index, const QString &sound, const QString &name, const WbSoundClip **clip) {
  if (sound.isEmpty()) {
    *clip = NULL;
    return;
  }

  const QString completeUrl = WbUrl::computePath(this, gUrlNames[index], sound, true);
  if (WbUrl::isWeb(completeUrl)) {
    if (mDownloader[index] && !mDownloader[index]->error().isEmpty()) {
      warn(mDownloader[index]->error());  // failure downloading or file does not exist (404)
      *clip = NULL;
      // downloader needs to be deleted in case the URL is switched back to something valid
      delete mDownloader[index];
      mDownloader[index] = NULL;
      return;
    }
    if (!WbNetwork::instance()->isCachedWithMapUpdate(completeUrl)) {
      downloadAsset(completeUrl, index);  // changed by supervisor
      return;
    }
  }

  WbSoundEngine::clearAllContactSoundSources();
  // determine extension from URL since for remotely defined assets the cached version doesn't retain this information
  const QString extension = sound.mid(sound.lastIndexOf('.') + 1).toLower();

  if (WbUrl::isWeb(completeUrl)) {
    assert(WbNetwork::instance()->isCachedNoMapUpdate(completeUrl));  // by this point, the asset should be cached
    *clip = WbSoundEngine::sound(WbNetwork::instance()->get(completeUrl), extension);
  }
  // completeUrl can contain missing_texture.png if the user inputs an invalid .png or .jpg file in the field. In this case,
  // clip should be set to NULL
  else if (!(completeUrl.isEmpty() || completeUrl == WbUrl::missingTexture()))
    *clip = WbSoundEngine::sound(WbUrl::computePath(this, name, completeUrl, true), extension);
  else
    *clip = NULL;
}

void WbContactProperties::updateBumpSound() {
  loadSound(0, mBumpSound->value(), "bumpSound", &mBumpSoundClip);
}

void WbContactProperties::updateRollSound() {
  loadSound(1, mRollSound->value(), "rollSound", &mRollSoundClip);
}

void WbContactProperties::updateSlideSound() {
  loadSound(2, mSlideSound->value(), "slideSound", &mSlideSoundClip);
}

void WbContactProperties::enableBodies() {
  // when a value is changed, we need to re-enable the bodies,
  // otherwise the new value is not applied in WbSimulationCluster::fillSurfaceParameters
  WbWorld::instance()->awake();
}
