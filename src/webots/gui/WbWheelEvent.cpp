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

#include "WbWheelEvent.hpp"

#include "WbEditCommand.hpp"
#include "WbSolid.hpp"
#include "WbUndoStack.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWorldInfo.hpp"

#define SIGN(x) ((x) > 0.0 ? 1.0 : -1.0)

// WbWheelEvent functions

WbWheelEvent::WbWheelEvent() {
}

// WbWheelLiftSolidEvent functions

WbWheelLiftSolidEvent::WbWheelLiftSolidEvent(WbViewpoint *viewpoint, WbSolid *selectedSolid) :
  mViewpoint(viewpoint),
  mSelectedSolid(selectedSolid),
  mInitialTranslation(selectedSolid->translation()) {
  mScaleFactor = WbWorld::instance()->worldInfo()->lineScale();
  mUpWorldVector = WbWorld::instance()->worldInfo()->upVector();
  mViewpoint->lock();
  mSelectedSolid->pausePhysics();
}

WbWheelLiftSolidEvent::~WbWheelLiftSolidEvent() {
  mViewpoint->unlock();
  if (mInitialTranslation != mSelectedSolid->translation())
    WbWorld::instance()->setModified();
  WbUndoStack::instance()->push(new WbEditCommand(mSelectedSolid->translationFieldValue(), WbVariant(mInitialTranslation),
                                                  WbVariant(mSelectedSolid->translationFieldValue()->variantValue())));
  mSelectedSolid->resumePhysics();
}

void WbWheelLiftSolidEvent::apply(int delta) {
  mSelectedSolid->setTranslation(SIGN(delta) * mScaleFactor * mUpWorldVector + mSelectedSolid->translation());
  mSelectedSolid->resetPhysics();
}
