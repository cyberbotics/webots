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

#include "WbLensFlare.hpp"

#include "WbFieldChecker.hpp"
#include "WbLight.hpp"
#include "WbMFVector2.hpp"
#include "WbNodeUtilities.hpp"
#include "WbRay.hpp"
#include "WbSFBool.hpp"
#include "WbSFDouble.hpp"
#include "WbWorld.hpp"
#include "WbWrenLensFlare.hpp"

void WbLensFlare::init() {
  mTransparency = findSFDouble("transparency");
  mScale = findSFDouble("scale");
  mBias = findSFDouble("bias");
  mDispersal = findSFDouble("dispersal");
  mHaloWidth = findSFDouble("haloWidth");
  mChromaDistortion = findSFDouble("chromaDistortion");
  mSamples = findSFInt("samples");
  mBlurIterations = findSFInt("blurIterations");
}

WbLensFlare::WbLensFlare(WbTokenizer *tokenizer) : WbBaseNode("LensFlare", tokenizer) {
  init();
}

WbLensFlare::WbLensFlare(const WbLensFlare &other) : WbBaseNode(other) {
  init();
}

WbLensFlare::WbLensFlare(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbLensFlare::~WbLensFlare() {
  qDeleteAll(mWrenLensFlares);
  mWrenLensFlares.clear();
}

void WbLensFlare::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mTransparency, &WbSFDouble::changed, this, &WbLensFlare::updateTransparency);
  connect(mScale, &WbSFDouble::changed, this, &WbLensFlare::updateScale);
  connect(mBias, &WbSFDouble::changed, this, &WbLensFlare::updateBias);
  connect(mDispersal, &WbSFDouble::changed, this, &WbLensFlare::updateDispersal);
  connect(mHaloWidth, &WbSFDouble::changed, this, &WbLensFlare::updateHaloWidth);
  connect(mChromaDistortion, &WbSFDouble::changed, this, &WbLensFlare::updateChromaDistortion);
  connect(mSamples, &WbSFInt::changed, this, &WbLensFlare::updateSamples);
  connect(mBlurIterations, &WbSFInt::changed, this, &WbLensFlare::updateBlur);
}

void WbLensFlare::setup(WrViewport *viewport) {
  WbWrenLensFlare *lensFlare = mWrenLensFlares.value(viewport, NULL);
  if (!lensFlare) {
    lensFlare = new WbWrenLensFlare();
    mWrenLensFlares.insert(viewport, lensFlare);
  }
  lensFlare->setup(viewport);

  updateTransparency();
  updateScale();
  updateBias();
  updateDispersal();
  updateHaloWidth();
  updateChromaDistortion();
  updateSamples();
  updateBlur();
}

void WbLensFlare::detachFromViewport() {
  QMapIterator<WrViewport *, WbWrenLensFlare *> it(mWrenLensFlares);
  while (it.hasNext()) {
    it.next();
    it.value()->detachFromViewport();
  }
  qDeleteAll(mWrenLensFlares);
  mWrenLensFlares.clear();
}

void WbLensFlare::updateTransparency() {
  if (WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mTransparency, 0.0, 1.0, 0.5))
    return;

  QMapIterator<WrViewport *, WbWrenLensFlare *> it(mWrenLensFlares);
  while (it.hasNext()) {
    it.next();
    it.value()->setTransparency(static_cast<float>(mTransparency->value()));
  }
}

void WbLensFlare::updateScale() {
  QMapIterator<WrViewport *, WbWrenLensFlare *> it(mWrenLensFlares);
  while (it.hasNext()) {
    it.next();
    it.value()->setScale(static_cast<float>(mScale->value()));
  }
}

void WbLensFlare::updateBias() {
  QMapIterator<WrViewport *, WbWrenLensFlare *> it(mWrenLensFlares);
  while (it.hasNext()) {
    it.next();
    it.value()->setBias(static_cast<float>(mBias->value()));
  }
}

void WbLensFlare::updateDispersal() {
  QMapIterator<WrViewport *, WbWrenLensFlare *> it(mWrenLensFlares);
  while (it.hasNext()) {
    it.next();
    it.value()->setDispersal(static_cast<float>(mDispersal->value()));
  }
}

void WbLensFlare::updateHaloWidth() {
  if (WbFieldChecker::resetDoubleIfNonPositive(this, mHaloWidth, 0.5))
    return;

  QMapIterator<WrViewport *, WbWrenLensFlare *> it(mWrenLensFlares);
  while (it.hasNext()) {
    it.next();
    it.value()->setHaloWidth(static_cast<float>(mHaloWidth->value()));
  }
}

void WbLensFlare::updateChromaDistortion() {
  QMapIterator<WrViewport *, WbWrenLensFlare *> it(mWrenLensFlares);
  while (it.hasNext()) {
    it.next();
    it.value()->setChromaDistortion(static_cast<float>(mChromaDistortion->value()));
  }
}

void WbLensFlare::updateSamples() {
  if (WbFieldChecker::resetIntIfNegative(this, mSamples, 1))
    return;

  QMapIterator<WrViewport *, WbWrenLensFlare *> it(mWrenLensFlares);
  while (it.hasNext()) {
    it.next();
    it.value()->setSamples(mSamples->value());
  }
}

void WbLensFlare::updateBlur() {
  if (WbFieldChecker::resetIntIfNegative(this, mBlurIterations, 2))
    return;

  QMapIterator<WrViewport *, WbWrenLensFlare *> it(mWrenLensFlares);
  while (it.hasNext()) {
    it.next();
    it.value()->setBlurIterations(mBlurIterations->value());
  }
}
