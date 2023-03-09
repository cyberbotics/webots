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

#include "WbWrenOpenGlContext.hpp"

#include <wren/gl_state.h>

#include <QtCore/QStack>

WbWrenOpenGlContext *WbWrenOpenGlContext::mWrenContext;
QSurface *WbWrenOpenGlContext::mWrenSurface;
bool WbWrenOpenGlContext::mIsCurrent;
QStack<bool> WbWrenOpenGlContext::mPreviousState;

void WbWrenOpenGlContext::destroy() {
  assert(mPreviousState.empty());
  delete mWrenContext;
}

// Makes WREN's OpenGL context current and marks it as active.
// WREN will immediately apply changes to OpenGL state until the context has been marked as inactive.
bool WbWrenOpenGlContext::makeWrenCurrent() {
  assert(mWrenContext);
  assert(mWrenSurface);

  mPreviousState.push(mIsCurrent);
  wr_gl_state_set_context_active(true);
  mIsCurrent = true;

  return mWrenContext->forceMakeCurrent(mWrenSurface);
}

// Marks WREN's OpenGL context as inactive, preventing WREN from making OpenGL calls until it is marked as active again.
// Any call to makeWrenCurrent should be followed by a call to doneWren before any possible OpenGL context changes.
void WbWrenOpenGlContext::doneWren() {
  if (!mPreviousState.empty() && mPreviousState.pop())
    return;

  wr_gl_state_set_context_active(false);
  mIsCurrent = false;
}

bool WbWrenOpenGlContext::isCurrent() {
  return mPreviousState.size() > 0;
}
