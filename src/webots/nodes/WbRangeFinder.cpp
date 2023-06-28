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

#include "WbRangeFinder.hpp"

#include "WbDataStream.hpp"
#include "WbFieldChecker.hpp"
#include "WbPreferences.hpp"
#include "WbRgb.hpp"
#include "WbWrenCamera.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenTextureOverlay.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>

void WbRangeFinder::init() {
  mCharType = 'r';

  // Fields initialization
  mMinRange = findSFDouble("minRange");
  mMaxRange = findSFDouble("maxRange");
  mResolution = findSFDouble("resolution");

  // backward compatibility
  WbSFBool *sphericalField = findSFBool("spherical");
  if (sphericalField->value()) {  // Deprecated in Webots R2023
    parsingWarn("Deprecated 'spherical' field, please use the 'projection' field instead.");
    if (isPlanarProjection())
      mProjection->setValue("cylindrical");
    sphericalField->setValue(false);
  }
}

WbRangeFinder::WbRangeFinder(WbTokenizer *tokenizer) : WbAbstractCamera("RangeFinder", tokenizer) {
  init();
}

WbRangeFinder::WbRangeFinder(const WbRangeFinder &other) : WbAbstractCamera(other) {
  init();
}

WbRangeFinder::WbRangeFinder(const WbNode &other) : WbAbstractCamera(other) {
  init();
}

WbRangeFinder::~WbRangeFinder() {
}

void WbRangeFinder::preFinalize() {
  WbAbstractCamera::preFinalize();

  updateNear();
  updateMinRange();
  updateMaxRange();
  updateResolution();
}

void WbRangeFinder::postFinalize() {
  WbAbstractCamera::postFinalize();

  connect(mNear, &WbSFDouble::changed, this, &WbRangeFinder::updateNear);
  connect(mMinRange, &WbSFDouble::changed, this, &WbRangeFinder::updateMinRange);
  connect(mMaxRange, &WbSFDouble::changed, this, &WbRangeFinder::updateMaxRange);
  connect(mResolution, &WbSFDouble::changed, this, &WbRangeFinder::updateResolution);
}

void WbRangeFinder::updateOrientation() {
  if (hasBeenSetup()) {
    // FLU axis orientation
    mWrenCamera->rotateRoll(M_PI_2);
    mWrenCamera->rotateYaw(-M_PI_2);
  }
}

void WbRangeFinder::initializeImageMemoryMappedFile() {
  WbAbstractCamera::initializeImageMemoryMappedFile();
  if (mImageMemoryMappedFile) {
    // initialize the memory mapped file with a black image
    float *im = rangeFinderImage();
    const int s = width() * height();
    for (int i = 0; i < s; i++)
      im[i] = 0.0f;
  }
}

QString WbRangeFinder::pixelInfo(int x, int y) const {
  WbRgb color;
  if (hasBeenSetup())
    color = mWrenCamera->copyPixelColourValue(x, y);
  return QString::asprintf("depth(%d,%d)=%f", x, y, color.red());
}

void WbRangeFinder::addConfigureToStream(WbDataStream &stream, bool reconfigure) {
  WbAbstractCamera::addConfigureToStream(stream, reconfigure);
  stream << (double)mMaxRange->value();
}

void WbRangeFinder::handleMessage(QDataStream &stream) {
  unsigned char command;
  stream >> command;

  if (WbAbstractCamera::handleCommand(stream, command))
    return;

  assert(0);
}

float *WbRangeFinder::rangeFinderImage() const {
  return reinterpret_cast<float *>(image());
}

void WbRangeFinder::createWrenCamera() {
  WbAbstractCamera::createWrenCamera();
  applyCameraSettings();
  applyMaxRangeToWren();
  applyResolutionToWren();

  updateOrientation();
  connect(mWrenCamera, &WbWrenCamera::cameraInitialized, this, &WbRangeFinder::updateOrientation);
}

/////////////////////
//  Update methods //
/////////////////////

void WbRangeFinder::updateNear() {
  if (WbFieldChecker::resetDoubleIfNonPositive(this, mNear, 0.01))
    return;

  if (mNear->value() > mMinRange->value()) {
    parsingWarn(tr("'near' is greater than to 'minRange'. Setting 'near' to %1.").arg(mMinRange->value()));
    mNear->setValueNoSignal(mMinRange->value());
    return;
  }

  if (hasBeenSetup())
    applyNearToWren();
}

void WbRangeFinder::updateMinRange() {
  if (WbFieldChecker::resetDoubleIfNonPositive(this, mMinRange, 0.01))
    return;

  if (mMinRange->value() < mNear->value()) {
    parsingWarn(tr("'minRange' is less than 'near'. Setting 'minRange' to %1.").arg(mNear->value()));
    mMinRange->setValueNoSignal(mNear->value());
  }
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
  }

  mNeedToConfigure = true;

  if (hasBeenSetup())
    applyMinRangeToWren();

  if (areWrenObjectsInitialized()) {
    applyFrustumToWren();
    if (isPlanarProjection() && hasBeenSetup())
      updateFrustumDisplay();
  }
}

void WbRangeFinder::updateMaxRange() {
  if (mMaxRange->value() <= mMinRange->value()) {
    double newMaxRange = mMinRange->value() + 1.0;
    parsingWarn(tr("'maxRange' is less or equal to 'minRange'. Setting 'maxRange' to %1.").arg(newMaxRange));
    mMaxRange->setValueNoSignal(newMaxRange);
  }

  mNeedToConfigure = true;

  if (hasBeenSetup())
    applyMaxRangeToWren();

  if (areWrenObjectsInitialized())
    applyFrustumToWren();
}

void WbRangeFinder::updateResolution() {
  if (WbFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0))
    return;

  if (hasBeenSetup())
    applyResolutionToWren();
}

bool WbRangeFinder::isFrustumEnabled() const {
  return WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_RANGE_FINDER_FRUSTUMS);
}

void WbRangeFinder::updateFrustumDisplayIfNeeded(int optionalRendering) {
  if (optionalRendering == WbWrenRenderingContext::VF_RANGE_FINDER_FRUSTUMS)
    updateFrustumDisplay();
}

/////////////////////
//  Apply methods  //
/////////////////////

void WbRangeFinder::applyResolutionToWren() {
  mWrenCamera->setRangeResolution(mResolution->value());
}

void WbRangeFinder::applyMinRangeToWren() {
  mWrenCamera->setMinRange(mMinRange->value());
}

void WbRangeFinder::applyMaxRangeToWren() {
  // camera
  mWrenCamera->setMaxRange(mMaxRange->value());
  // overlay
  if (mOverlay)
    mOverlay->setMaxRange(mMaxRange->value());
}

void WbRangeFinder::applyCameraSettingsToWren() {
  WbAbstractCamera::applyCameraSettingsToWren();
  applyResolutionToWren();
  applyMaxRangeToWren();
}

int WbRangeFinder::textureGLId() const {
  if (mWrenCamera)
    return mWrenCamera->textureGLId();
  return WbRenderingDevice::textureGLId();
}
