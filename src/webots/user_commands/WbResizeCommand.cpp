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

#include "WbResizeCommand.hpp"
#include "WbGeometry.hpp"

#include <cassert>

WbResizeCommand::WbResizeCommand(WbGeometry *geometry, const WbVector3 &scale, QUndoCommand *parent) :
  QUndoCommand(parent),
  mGeometry(geometry),
  mScale(scale),
  mInvScale(1.0 / scale.x(), 1.0 / scale.y(), 1.0 / scale.z()) {
  mIsFirstCall = true;
  setText(QObject::tr("rescale"));
}

void WbResizeCommand::undo() {
  resetValue(mInvScale);
}

void WbResizeCommand::redo() {
  if (mIsFirstCall) {
    mIsFirstCall = false;
    return;
  }

  resetValue(mScale);
}

void WbResizeCommand::resetValue(const WbVector3 &scale) {
  assert(mGeometry);
  mGeometry->rescale(scale);
}
