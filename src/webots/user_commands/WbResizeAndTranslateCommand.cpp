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

#include "WbResizeAndTranslateCommand.hpp"
#include "WbIndexedFaceSet.hpp"

#include <cassert>

WbResizeAndTranslateCommand::WbResizeAndTranslateCommand(WbGeometry *geometry, const WbVector3 &scale,
                                                         const WbVector3 &translation, QUndoCommand *parent) :
  WbResizeCommand(geometry, scale, parent),
  mTranslation(translation) {
  mIsTranslationSet = (mTranslation != WbVector3());
}

void WbResizeAndTranslateCommand::undo() {
  if (!mIsTranslationSet)
    WbResizeCommand::undo();
  else
    resetValue(true);
}

void WbResizeAndTranslateCommand::redo() {
  if (!mIsTranslationSet) {
    WbResizeCommand::redo();
    return;
  }

  if (mIsFirstCall) {
    mIsFirstCall = false;
    return;
  }

  resetValue(false);
}

void WbResizeAndTranslateCommand::resetValue(bool invertedAction) {
  assert(mGeometry);

  WbIndexedFaceSet *indexedFaceSet = dynamic_cast<WbIndexedFaceSet *>(mGeometry);
  if (!indexedFaceSet)
    return;

  if (invertedAction) {
    indexedFaceSet->translate(-mTranslation);
    indexedFaceSet->rescale(mInvScale);
  } else
    indexedFaceSet->rescaleAndTranslate(mScale, mTranslation);
}
